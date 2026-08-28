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

"""Genome v2 list + scene spec -> .omniworld.

Supersedes worldgen.py. Writes one `controller "<none>"` Robot per genome,
built from capsules: a torso along +x, a visual-only head, and 1-3 bilateral
limb pairs of 1-2 hinged segments each. The arena/lighting/food/director VRML
comes from scene.py (owner C) as `scene_lines`; `_fallback_scene()` here is a
minimal stand-in so this module and probe_v2.py never depend on it.

Every load-bearing field here was established by the probes in ../README.md.
The ones that are NOT guessable, and silently produce a wrong-but-passing world
if omitted:

  * `controller "<none>"`            -- "" and "void" each spawn a real process
  * `WorldInfo.newtonRobotColliders` -- else the torso collider becomes a 1 mm sphere
  * explicit inertiaMatrix + centerOfMass on the Robot root
  * minPosition/maxPosition on every motor, wider than the gait's bias+amp
  * floor as a Box, never a Plane
  * every creature wrapped in Pose{} so births do not perturb the population

CAPSULE AXIS. DESIGN_v2.md says the Capsule axis is Y (upstream-Webots lore);
in THIS engine it is Z: resources/nodes/Capsule.wrl ("aligned along the local
z-axis"), OmCapsule.cpp:136, and the Newton runtime's add_shape_capsule
("OmCapsule is Z-aligned; so is a newton capsule"). So the torso capsule is
turned Z->X with `Pose { rotation 0 1 0 1.5708 }`, and a limb segment, which
hangs along its own local -z, needs only a translation Pose. The rest-height
gate in probe_v2.py is what proves this: a wrong axis puts the torso at the
wrong height, not in the log.
"""
import math
import os

from . import genome2 as G

DEFAULT_SPACING = 3.0
PARK_XY = (60.0, 60.0)    # dead-slot crypt: a static slab (scene.py CRYPT) they rest on
PARK_Z = 0.6              # dropped onto the slab; at rest a revive is a plain teleport
WALL_H = 0.4
WALL_T = 0.2
FLOOR_T = 0.1
FLOOR_COLOUR = (0.16, 0.18, 0.15)
WALL_COLOUR = (0.30, 0.29, 0.27)


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
    """Saturated hues. Washed-out pastels are indistinguishable against the floor."""
    h %= 1.0
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p, q, t = v * (1 - s), v * (1 - s * f), v * (1 - s * (1 - f))
    return [(v, t, p), (q, v, p), (p, v, t),
            (p, q, v), (t, p, v), (v, p, q)][i]


def _pbr(rgb, rough=0.75, metal=0.0):
    return ('PBRAppearance { baseColor %.3f %.3f %.3f roughness %.2f metalness %.2f }'
            % (rgb[0], rgb[1], rgb[2], rough, metal))


def joint_defs(idx, g):
    """The (pair, side, joint) tuples the writer authors for genome g, with
    their DEF names, in authoring order. The director resolves exactly these."""
    out = []
    for k, p in enumerate(g["body"]["pairs"]):
        for side in G.SIDES:
            for s_i in range(len(p["segments"])):
                jn = "H" if s_i == 0 else "K"
                out.append({"pair": k, "side": side, "joint": G.JOINTS[s_i],
                            "def": "C%d_P%d_%s_%s" % (idx, k, side, jn)})
    return out


def _segment_solid(idx, k, side, s_i, segs, splay, colour, indent, anchor):
    """One limb segment Solid (recursive for the knee). The Solid sits AT its
    joint anchor; its capsule hangs along local -z, centre at -length/2, and
    the next segment hangs from -length."""
    seg = segs[s_i]
    L, r = seg["length"], seg["radius"]
    m = G.capsule_mass(L, r)
    axial, trans, _ = G.capsule_inertia(m, L, r)
    jn = "H" if s_i == 0 else "K"
    dname = "C%d_P%d_%s_%s" % (idx, k, side, jn)
    P = " " * indent
    out = []
    A = out.append
    A(P + 'endPoint DEF %s_LINK Solid {' % dname)
    A(P + '  translation %.5f %.5f %.5f' % anchor)
    if s_i == 0:
        # Hip: rolled about x by +-splay so the limb hangs down-and-outward
        # (+y is the L side). The knee inherits this frame, so its axis and
        # capsule are tilted by the same splay -- intended.
        A(P + '  rotation 1 0 0 %.5f' % (splay if side == "L" else -splay))
    A(P + '  name "%s_link"' % dname.lower())
    A(P + '  children [')
    A(P + '    Pose {')
    A(P + '      translation 0 0 %.5f' % (-L / 2.0))
    A(P + '      children [ Shape { appearance %s geometry Capsule { height %.5f radius %.5f } } ]'
      % (_pbr(colour, 0.85), L, r))
    A(P + '    }')
    if s_i + 1 < len(segs):
        out += _hinge(idx, k, side, s_i + 1, segs, splay, colour, indent + 4)
    A(P + '  ]')
    A(P + '  boundingObject Pose {')
    A(P + '    translation 0 0 %.5f' % (-L / 2.0))
    A(P + '    children [ Capsule { height %.5f radius %.5f } ]' % (L, r))
    A(P + '  }')
    # Explicit tensor about the capsule centre (a Pose-offset collider is a
    # single primitive, so auto-inertia would also work; explicit is exact and
    # independent of the composer's Pose handling).
    A(P + '  physics Physics {')
    A(P + '    density -1')
    A(P + '    mass %.5f' % m)
    A(P + '    centerOfMass [ 0 0 %.5f ]' % (-L / 2.0))
    A(P + '    inertiaMatrix [ %.6g %.6g %.6g, 0 0 0 ]' % (trans, trans, axial))
    A(P + '  }')
    A(P + '}')
    return out


def _hinge(idx, k, side, s_i, segs, splay, colour, indent, anchor=(0.0, 0.0, 0.0)):
    """HingeJoint + its motor/sensor + the segment Solid it drives."""
    jn = "H" if s_i == 0 else "K"
    dname = "C%d_P%d_%s_%s" % (idx, k, side, jn)
    if s_i > 0:
        anchor = (0.0, 0.0, -segs[s_i - 1]["length"])
    P = " " * indent
    out = []
    A = out.append
    A(P + 'DEF %s HingeJoint {' % dname)
    A(P + '  jointParameters DEF %s_PARAMS HingeJointParameters {' % dname)
    # Pitch axis for BOTH sides: fore/aft swing is the thrust axis. For the
    # knee this is segment 1's y, i.e. tilted by the splay -- intended.
    A(P + '    axis 0 1 0')
    A(P + '    anchor %.5f %.5f %.5f' % anchor)
    A(P + '    position 0')
    A(P + '    minStop -2')
    A(P + '    maxStop 2')
    A(P + '  }')
    A(P + '  device [')
    # Without min/maxPosition the joint is built as a ke=0 velocity wheel and
    # every position target is silently ignored.
    A(P + '    RotationalMotor {')
    A(P + '      name "%s_motor"' % dname.lower())
    A(P + '      minPosition -%.2f' % G.JOINT_LIMIT)
    A(P + '      maxPosition %.2f' % G.JOINT_LIMIT)
    A(P + '      maxTorque 1.5')
    A(P + '      maxVelocity 4')
    A(P + '    }')
    A(P + '    PositionSensor { name "%s" }' % dname.lower())
    A(P + '  ]')
    out += _segment_solid(idx, k, side, s_i, segs, splay, colour, indent + 2, anchor)
    A(P + '}')
    return out


def creature_vrml(idx, g, pos, yaw=0.0):
    """One creature as VRML lines (NOT Pose-wrapped; write_world does that).
    idx is the slot: DEF CREATURE_{idx}, joints C{idx}_P{k}_{L|R}_{H|K}."""
    body = g["body"]
    torso = body["torso"]
    L, R = torso["length"], torso["radius"]
    m_t = G.torso_mass(body)
    axial, trans, _ = G.capsule_inertia(m_t, L, R)
    hue = body["hue"]
    skin = hsv(hue)
    limb = tuple(c * 0.62 for c in skin)
    eye = (0.04, 0.04, 0.05)
    hr = body["head"]["radius"]
    head_c = (L / 2.0 + 0.55 * hr, 0.0, 0.35 * R)

    out = []
    A = out.append
    A('DEF CREATURE_%d Robot {' % idx)
    A('  translation %.4f %.4f %.4f' % tuple(pos))
    A('  rotation 0 0 1 %.5f' % yaw)
    A('  name "creature_%d"' % idx)
    A('  controller "<none>"')
    A('  children [')
    # Torso capsule turned Z->X (see CAPSULE AXIS in the module docstring).
    A('    Pose {')
    A('      rotation 0 1 0 1.5708')
    A('      children [ Shape { appearance %s geometry Capsule { height %.5f radius %.5f } } ]'
      % (_pbr(skin), L, R))
    A('    }')
    # Head + eyes: VISUAL ONLY (no Solid, no collider) -- part of the torso body.
    A('    Pose {')
    A('      translation %.4f %.4f %.4f' % head_c)
    A('      children [')
    A('        Shape { appearance %s geometry Sphere { radius %.4f subdivision 3 } }'
      % (_pbr(skin), hr))
    for sy in (1.0, -1.0):
        A('        Pose { translation %.4f %.4f %.4f children [ Shape { appearance %s '
          'geometry Sphere { radius %.4f subdivision 2 } } ] }'
          % (0.62 * hr, sy * 0.55 * hr, 0.38 * hr, _pbr(eye, 0.3), 0.22 * hr))
    A('      ]')
    A('    }')

    for k, p in enumerate(body["pairs"]):
        for side in G.SIDES:
            sy = 1.0 if side == "L" else -1.0
            anchor = (p["x"] * L / 2.0, sy * R, p["z"])
            out += _hinge(idx, k, side, 0, p["segments"], p["splay"], limb, 4,
                          anchor=anchor)

    A('  ]')
    A('  boundingObject Pose {')
    A('    rotation 0 1 0 1.5708')
    A('    children [ Capsule { height %.5f radius %.5f } ]' % (L, R))
    A('  }')
    A('  physics Physics {')
    A('    density -1')
    A('    mass %.5f' % m_t)
    A('    centerOfMass [ 0 0 0 ]')
    A('    inertiaMatrix [ %.6g %.6g %.6g, 0 0 0 ]' % (axial, trans, trans))
    A('  }')
    A('}')
    return out


# ------------------------------------------------------------------ scene
def static_box(defname, name, pos, size, colour=WALL_COLOUR):
    return [
        'DEF %s Solid {' % defname,
        '  translation %.3f %.3f %.3f' % pos,
        '  name "%s"' % name,
        '  children [',
        '    Shape {',
        '      appearance %s' % _pbr(colour, 1.0),
        '      geometry Box { size %.3f %.3f %.3f }' % size,
        '    }',
        '  ]',
        '  boundingObject Box { size %.3f %.3f %.3f }' % size,
        '}',
    ]


def _fallback_scene(arena, controller="terrarium_life"):
    """Minimal arena when scene.py is not supplied: floor, 4 walls, canonical
    lighting, hero Viewpoint, director. Matches the contract's Scene section
    minus the food pool."""
    L = []
    A = L.append
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
    L += static_box('FLOOR', 'arena_floor', (0, 0, -FLOOR_T / 2.0),
                    (arena, arena, FLOOR_T), FLOOR_COLOUR)
    h = arena / 2.0 + WALL_T / 2.0
    span = arena + 2.0 * WALL_T
    for name, pos, size in (
            ('WALL_N', (0, h, WALL_H / 2.0), (span, WALL_T, WALL_H)),
            ('WALL_S', (0, -h, WALL_H / 2.0), (span, WALL_T, WALL_H)),
            ('WALL_E', (h, 0, WALL_H / 2.0), (WALL_T, span, WALL_H)),
            ('WALL_W', (-h, 0, WALL_H / 2.0), (WALL_T, span, WALL_H))):
        L += static_box(name, name.lower(), pos, size)
    A('')
    A('DEF DIRECTOR Robot {')
    A('  translation 0 0 %.2f' % (arena * 0.5))
    A('  name "director"')
    A('  controller "%s"' % controller)
    A('  supervisor TRUE')
    A('  synchronization TRUE')
    A('  children [ ]')
    A('}')
    return L


def grid_positions(n, arena, spacing=DEFAULT_SPACING, margin=1.5):
    """n cells on a centred grid that fits inside the arena walls."""
    if n == 0:
        return []
    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = int(math.ceil(n / cols))
    usable = arena - 2.0 * margin
    sp = min(spacing, usable / max(cols, rows, 1))
    out = []
    for i in range(n):
        cx, cy = i % cols, i // cols
        out.append(((cx - (cols - 1) / 2.0) * sp, (cy - (rows - 1) / 2.0) * sp))
    return out


def park_position(slot):
    """(60 + 2i, 60, 0.6): dropped onto the crypt slab, where it rests."""
    return (PARK_XY[0] + 2.0 * slot, PARK_XY[1], PARK_Z)


def placements(pop, arena, spacing=DEFAULT_SPACING):
    """Where each population entry is authored. Entries may carry `slot`
    (default: list index), `alive_at_start` (default True), `pos` [x, y] and
    `yaw`; the rest is filled from a centred grid and a golden-angle yaw."""
    out = []
    alive = [e for e in pop if e.get("alive_at_start", True) and e.get("pos") is None]
    grid = iter(grid_positions(len(alive), arena, spacing))
    for i, e in enumerate(pop):
        slot = int(e.get("slot", i))
        yaw = e.get("yaw")
        if yaw is None:
            yaw = ((slot * 2.39996) + math.pi) % (2.0 * math.pi) - math.pi
        if not e.get("alive_at_start", True):
            pos, parked, yaw = park_position(slot), True, 0.0
        else:
            xy = tuple(e["pos"]) if e.get("pos") is not None else next(grid)
            pos, parked = (xy[0], xy[1], G.spawn_z(e["body"])), False
        out.append({"slot": slot, "def": "CREATURE_%d" % slot, "pos": list(pos),
                    "yaw": yaw, "parked": parked, "id": e["id"],
                    "species": e["species"]})
    slots = [p["slot"] for p in out]
    if len(set(slots)) != len(slots):
        raise ValueError("duplicate slots: %s" % sorted(slots))
    return out


def write_world(pop, path, scene_lines=None, controller="terrarium_life",
                arena=14.0, spacing=DEFAULT_SPACING, ke=8000, kd=200,
                cone="elliptic", impratio=10, dt=8, title="Alife terrarium v2"):
    """Write pop (genome v2 dicts, optionally carrying slot/alive_at_start/
    pos/yaw) to path. `scene_lines` is the arena/lighting/food/director VRML
    from scene.py; None selects _fallback_scene(). Physics config per the
    contract's hard facts -- fitness depends on it, change nowhere."""
    problems = {e["id"]: G.validate(e) for e in pop}
    problems = {k: v for k, v in problems.items() if v}
    if problems:
        raise ValueError("invalid genomes: %s" % problems)
    place = placements(pop, arena, spacing)

    L = []
    A = L.append
    A('#OMNISIM R2025a utf8')
    A('')
    A('# GENERATED by projects/alife/alife/worldgen2.py -- do not hand-edit.')
    A('# Population: %d creatures (%d parked).  See projects/alife/DESIGN_v2.md.'
      % (len(pop), sum(1 for p in place if p["parked"])))
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
    L += list(scene_lines) if scene_lines is not None else _fallback_scene(arena, controller)
    A('')

    hinges = 0
    for e, p in zip(pop, place):
        L += _pose_wrapped(creature_vrml(p["slot"], e, p["pos"], p["yaw"]))
        A('')
        hinges += G.joint_count(e)

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(L) + '\n')
    return {"path": path, "n": len(pop), "arena": arena, "hinges": hinges,
            "placements": place}
