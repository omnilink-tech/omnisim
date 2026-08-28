#!/usr/bin/env python3
"""The Metazoa cell: geometry constants, VRML emitter, docking placement.

Owner: A. Everyone else READS this module -- B's organism.py for the frame
and the axis convention, C's ecology.py for the DEF names it writes.

Frame (DESIGN.md "The cell"):

  * The TAIL block is the Robot root, a 0.06 m cube centred at the origin,
    x in [-0.03, 0.03]. Its Physics carries an explicit inertiaMatrix +
    centerOfMass because the geometry-derived path excludes OmRobot
    (projects/alife/README.md).
  * The HINGE sits at the seam x = +0.03, axis local +y, anchor (0.03, 0, 0).
  * The NOSE block is the hinge endPoint Solid, centre x = +0.06 (0.03..0.09).
  * Folded flat = 0 rad. SIGN: measured by probe_p1 (README.md "hinge sign")
    -- do not assume; the probe reports which way a positive target moves
    the nose in the tail's frame.
  * Four `Connector` faces, all "symmetric": f_tail on the tail's -x face,
    f_nose / f_left / f_right on the nose block's +x / +y / -y faces. A
    Connector's docking normal is its local +x (OmConnector::isXAlignedWith
    wants the two normals ANTI-parallel), so each face is yawed to point
    outward. Every Connector is a child of the Solid whose body it welds.
  * The charge ring is a visual-only Cylinder on top of the tail block; its
    DEF'd PBRAppearance (`CELL_<i>_RING_APP`) is what ecology writes.

Authoring convention (write_world): `Pose { translation X Y Z rotation 0 0 1
YAW children [ DEF CELL_<i> Robot { rotation 1 0 0 ROLL ... } ] }`. Yaw on
the Pose, roll on the Robot, so the cell's world rotation is Rz(yaw)*Rx(roll):
roll 0 -> the hinge axis is horizontal-transverse (a PITCH hinge); roll 90
deg -> the axis is vertical (a YAW hinge); roll 180 -> pitch again.

Docking geometry (DESIGN.md): two cells chain when j's tail face meets i's
nose face. `chain_placement` and `branch_placement` are THE definition of
where a docked cell sits -- worldgen authors from them and the probe measures
against them, so they can never disagree.

DEF names, all stamped with the cell index i:
  CELL_i (Robot)  CELL_i_NOSE (Solid)  CELL_i_HINGE  CELL_i_HINGE_PARAMS
  CELL_i_MOTOR  CELL_i_RING  CELL_i_RING_APP
  CELL_i_F_TAIL  CELL_i_F_NOSE  CELL_i_F_LEFT  CELL_i_F_RIGHT
  v2 (rollers=True): CELL_i_ROLL_T / CELL_i_ROLL_N (passive HingeJoints) and
  their CELL_i_ROLL_T_SOLID / CELL_i_ROLL_N_SOLID rollers.
  v3 (rollers="v3"): CELL_i_ROLL_{T,N}{B,S} (+ _SOLID): B = bottom (-z) face,
  S = the -y side face; 4 rollers, 6 bodies per cell.
"""
import math

# ---------------------------------------------------------------- geometry
BLOCK = 0.06                 # cube edge [m]
HALF = BLOCK / 2.0           # 0.03
MASS = 0.12                  # per block [kg]  (density ~555 kg/m^3)
INERTIA = MASS * BLOCK * BLOCK / 6.0      # 7.2e-5, solid cube about its centre
HINGE_X = HALF               # seam
NOSE_X = BLOCK               # nose centre in the tail frame
CELL_LENGTH = 2.0 * BLOCK    # tail face (-0.03) to nose face (+0.09)
TAIL_FACE_X = -HALF          # f_tail origin
NOSE_FACE_X = NOSE_X + HALF  # f_nose origin in the TAIL frame (0.09)
REST_Z = HALF                # centre height of a cube resting on z = 0
SPAWN_Z = REST_Z + 0.002     # authored 2 mm up so it settles, never starts inside the floor
RING_HEIGHT = 0.004
RING_RADIUS = 0.022
RING_Z = HALF + RING_HEIGHT / 2.0

# cell v2: passive belly rollers (P1b). One free HingeJoint per block on
# its bottom face, axis local +y, carrying a light cylinder that protrudes
# ROLLER_PROTRUDE below the block: the cell rolls along its x and resists
# y -- the ACM snake-robot answer to isotropic floor friction. Rollers of a
# yaw-rolled cell point sideways and never touch the floor (intended). The
# engine substitutes cylinder colliders with capsules of the same radius,
# so the protrusion is unchanged.
ROLLER_RADIUS = 0.012
ROLLER_HEIGHT = 0.04
ROLLER_MASS = 0.01
ROLLER_PROTRUDE = 0.004
ROLLER_Z = -HALF + ROLLER_RADIUS - ROLLER_PROTRUDE     # -0.022 in the block frame
ROLLER_I_AXIAL = 0.5 * ROLLER_MASS * ROLLER_RADIUS ** 2
ROLLER_I_TRANS = ROLLER_MASS / 12.0 * (3.0 * ROLLER_RADIUS ** 2 + ROLLER_HEIGHT ** 2)
REST_Z_V2 = HALF + ROLLER_PROTRUDE                     # 0.034: block centre on rollers
MAX_TORQUE_V2 = 0.35        # P1: upright at 0.2, tips at 0.6 (README)
ROLLER_DAMPING = 0.002      # bearing damping [N m s] (P1b: a frictionless bearing coasted 5.86 m)
# cell v3 (P1c): a second roller pair on the -y SIDE face of both blocks,
# axle = the block's local z at local (0, -ROLLER_Y, 0), so a cell rolled
# +90 deg about x (dock_rotation 1, local -y -> world -z) rides on rollers
# whose axle runs along world y: it rolls along the spine and resists the
# lateral push exactly as the bottom pair does for a pitch cell. The unused
# pair points sideways / up and never touches (intended). Dock rotations
# are restricted to {0, 1}: roll 2 or 3 would put every roller in the air.
ROLLER_Y = -ROLLER_Z        # 0.022: side-roller offset along -y
# HingeJointParameters.dampingConstant is UNIMPLEMENTED under Newton
# (OmHingeJoint.cpp:210: it was an ODE AMotor and nothing replaced it), so
# the bearing is realised as a limitless RotationalMotor that is never
# commanded: the engine builds it as a ke=0 velocity wheel (kd 500) whose
# target velocity is 0, i.e. -kd*w clamped to maxTorque = a Coulomb brake
# of ROLLER_BRAKE_TORQUE on the axle. The dampingConstant field is still
# written to document the intent.
ROLLER_BRAKE_TORQUE = 0.001  # N m per roller

# hinge
HINGE_MIN = -2.6             # RotationalMotor min/maxPosition (needed: no limits = ke 0 wheel)
HINGE_MAX = 2.6
HINGE_STOP = 2.8             # HingeJointParameters min/maxStop
MAX_TORQUE = 0.6
MAX_VELOCITY = 5.0

# connector faces (DESIGN.md)
DISTANCE_TOLERANCE = 0.03
AXIS_TOLERANCE = 0.45
ROTATION_TOLERANCE = 0.45
NUMBER_OF_ROTATIONS = 4
DEFAULT_GAP = 0.01           # face-to-face gap of a docked pair (< DISTANCE_TOLERANCE)

FACES = ("f_tail", "f_nose", "f_left", "f_right")
# (host, local translation, yaw about local z) -- host "tail" = Robot root,
# "nose" = the endPoint Solid. Normal = local +x after the yaw.
FACE_SPEC = {
    "f_tail":  ("tail", (-HALF, 0.0, 0.0), math.pi),
    "f_nose":  ("nose", (HALF, 0.0, 0.0), 0.0),
    "f_left":  ("nose", (0.0, HALF, 0.0), math.pi / 2.0),
    "f_right": ("nose", (0.0, -HALF, 0.0), -math.pi / 2.0),
}

DEFAULT_COLOUR = (0.85, 0.85, 0.80)
DEBRIS_COLOUR = (0.12, 0.11, 0.10)
RING_FULL = (0.0, 1.0, 0.6)


# ---------------------------------------------------------------- DEF names
def spawn_z(rollers=False):
    """Authored centre height: 2 mm above rest so a cell settles onto the
    floor (v1) or onto its rollers (v2) instead of starting inside it."""
    return (REST_Z_V2 if rollers else REST_Z) + 0.002


def def_name(i, part=None):
    base = "CELL_%d" % int(i)
    return base if part is None else "%s_%s" % (base, part)


def face_def(i, face):
    """`CELL_i_F_TAIL` etc. from a face name ('f_tail')."""
    return def_name(i, face.upper())


def cell_defs(i):
    """Every DEF a cell carries, for resolvers."""
    return {
        "robot": def_name(i), "nose": def_name(i, "NOSE"),
        "hinge": def_name(i, "HINGE"), "hinge_params": def_name(i, "HINGE_PARAMS"),
        "motor": def_name(i, "MOTOR"), "ring": def_name(i, "RING"),
        "ring_app": def_name(i, "RING_APP"),
        "faces": {f: face_def(i, f) for f in FACES},
    }


# ---------------------------------------------------------------- rotations
def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def cell_rotation(yaw, roll=0.0):
    """World rotation matrix of a cell authored with Pose yaw + Robot roll."""
    return mat_mul(rot_z(yaw), rot_x(roll))


def axis_angle(m):
    """Axis-angle (ax, ay, az, angle) of a rotation matrix, for a VRML
    `rotation` field."""
    tr = m[0][0] + m[1][1] + m[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return (0.0, 0.0, 1.0, 0.0)
    if abs(ang - math.pi) < 1e-6:
        ax = math.sqrt(max(0.0, (m[0][0] + 1.0) / 2.0))
        ay = math.sqrt(max(0.0, (m[1][1] + 1.0) / 2.0))
        az = math.sqrt(max(0.0, (m[2][2] + 1.0) / 2.0))
        if m[0][1] < 0:
            ay = -ay
        if m[0][2] < 0:
            az = -az
        return (ax, ay, az, ang)
    d = 2.0 * math.sin(ang)
    return ((m[2][1] - m[1][2]) / d, (m[0][2] - m[2][0]) / d, (m[1][0] - m[0][1]) / d, ang)


def hinge_axis_world(yaw, roll=0.0):
    """World direction of the hinge axis (local +y) for a cell at yaw/roll."""
    return mat_vec(cell_rotation(yaw, roll), [0.0, 1.0, 0.0])


def hinge_kind(roll):
    """'pitch' when the hinge axis is horizontal (roll ~ 0 or 180 deg), 'yaw'
    when vertical (roll ~ +-90 deg). Any other roll is 'tilted'."""
    q = (roll / (math.pi / 2.0)) % 4.0
    k = round(q)
    if abs(q - k) > 1e-6:
        return "tilted"
    return "pitch" if int(k) % 2 == 0 else "yaw"


# ---------------------------------------------------------------- docking geometry
def _pose_tuple(p):
    """Normalise a pose to (x, y, z, yaw, roll) from a 5-tuple, a dict, or a
    shorter tuple with defaults (z = SPAWN_Z, yaw = roll = 0)."""
    if isinstance(p, dict):
        pos = p.get("pos", (0.0, 0.0, SPAWN_Z))
        z = pos[2] if len(pos) > 2 else SPAWN_Z
        return (float(pos[0]), float(pos[1]), float(z),
                float(p.get("yaw", 0.0)), float(p.get("roll", 0.0)))
    p = list(p)
    if len(p) == 2:
        p.append(SPAWN_Z)
    p += [0.0] * (5 - len(p))
    return tuple(float(v) for v in p[:5])


def chain_placement(head_pose, k, gap=DEFAULT_GAP, dock_rotations=None):
    """Pose of the k-th cell of a chain whose head (k = 0) is at head_pose.

    Cell k's tail face meets cell k-1's nose face: its origin is the head's
    origin + R_head * (k * (CELL_LENGTH + gap), 0, 0), same yaw, rolled
    about the chain's x by 90 deg * dock_rotations[k].

    `dock_rotations[k]` is the roll of cell k RELATIVE TO THE HEAD, in quarter
    turns (0..3), so the P1 gate's "alternating chain, rotations [0,1,0,1]"
    is pitch, yaw, pitch, yaw. (A relative-to-previous reading would make
    [0,1,0,1] pitch, yaw, yaw, pitch, which is not alternating; the per-cell
    axis is `hinge_kind(roll)` either way.) None / short lists read as 0.

    Returns {"pos": [x, y, z], "yaw": yaw, "roll": roll}. Everything is in the
    head's frame, so a rolled head rolls the whole chain with it."""
    x, y, z, yaw, roll0 = _pose_tuple(head_pose)
    rot = 0
    if dock_rotations is not None and k < len(dock_rotations):
        rot = int(dock_rotations[k]) % 4
    R = cell_rotation(yaw, roll0)
    d = mat_vec(R, [k * (CELL_LENGTH + gap), 0.0, 0.0])
    return {"pos": [x + d[0], y + d[1], z + d[2]], "yaw": yaw,
            "roll": roll0 + rot * (math.pi / 2.0)}


def chain_poses(head_pose, n, gap=DEFAULT_GAP, dock_rotations=None):
    return [chain_placement(head_pose, k, gap, dock_rotations) for k in range(n)]


def branch_placement(cell_pose, side="L", gap=DEFAULT_GAP):
    """Pose of a branch cell whose tail face meets this cell's f_left ('L',
    the nose block's +y face) or f_right ('R', -y).

    Branch origin = nose centre + R * (0, +-(HALF + HALF + gap), 0); its +x is
    this cell's +-y, i.e. yawed +-90 deg in the parent's frame. The result
    carries `axis_angle` (exact for any parent roll) and `yaw`/`roll`, which
    are exact only when the parent's side faces are horizontal (roll a
    multiple of 180 deg); `exact` says which. A parent rolled 90 deg has its
    side faces pointing up/down, so a branch there would stand vertical."""
    x, y, z, yaw, roll = _pose_tuple(cell_pose)
    s = 1.0 if side.upper().startswith("L") else -1.0
    R = cell_rotation(yaw, roll)
    d = mat_vec(R, [NOSE_X, s * (BLOCK + gap), 0.0])
    Rb = mat_mul(R, rot_z(s * math.pi / 2.0))
    q = (roll / math.pi) % 2.0
    exact = abs(q - round(q)) < 1e-6
    if exact and int(round(q)) % 2 == 0:
        b_yaw, b_roll = yaw + s * math.pi / 2.0, roll
    elif exact:
        b_yaw, b_roll = yaw - s * math.pi / 2.0, roll   # Rx(pi) conjugates Rz(a) to Rz(-a)
    else:
        b_yaw, b_roll = yaw + s * math.pi / 2.0, roll
    return {"pos": [x + d[0], y + d[1], z + d[2]], "yaw": b_yaw, "roll": b_roll,
            "axis_angle": axis_angle(Rb), "exact": exact, "side": side.upper()[0]}


def face_world(cell_pose, face, hinge=0.0):
    """World position of a face's connector origin for a cell at cell_pose
    with its hinge at `hinge` rad (nose faces rotate with the nose block,
    right-hand about local +y). The analytic twin of getPosition() on the
    connector node, used by the probe."""
    x, y, z, yaw, roll = _pose_tuple(cell_pose)
    host, loc, _ = FACE_SPEC[face]
    R = cell_rotation(yaw, roll)
    if host == "tail":
        p = list(loc)
    else:
        c, s = math.cos(hinge), math.sin(hinge)
        Ry = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
        v = [NOSE_X - HINGE_X + loc[0], loc[1], loc[2]]
        w = mat_vec(Ry, v)
        p = [HINGE_X + w[0], w[1], w[2]]
    d = mat_vec(R, p)
    return [x + d[0], y + d[1], z + d[2]]


# ---------------------------------------------------------------- VRML
def pbr(rgb, rough=0.7, metal=0.05):
    return ('PBRAppearance { baseColor %.3f %.3f %.3f roughness %.2f metalness %.2f }'
            % (rgb[0], rgb[1], rgb[2], rough, metal))


def connector_lines(i, face, indent):
    """One `Connector` face. Fields per DESIGN.md. `snap FALSE`: the Newton
    weld engages at the current relative pose regardless (OmConnector.cpp --
    the snap chain has been a no-op since ODE's deletion)."""
    host, (tx, ty, tz), yaw = FACE_SPEC[face]
    P = " " * indent
    return [
        P + 'DEF %s Connector {' % face_def(i, face),
        P + '  name "%s"' % face,
        P + '  translation %.4f %.4f %.4f' % (tx, ty, tz),
        P + '  rotation 0 0 1 %.5f' % yaw,
        P + '  type "symmetric"',
        P + '  isLocked FALSE',
        P + '  autoLock FALSE',
        P + '  unilateralLock TRUE',
        P + '  unilateralUnlock TRUE',
        P + '  distanceTolerance %g' % DISTANCE_TOLERANCE,
        P + '  axisTolerance %g' % AXIS_TOLERANCE,
        P + '  rotationTolerance %g' % ROTATION_TOLERANCE,
        P + '  numberOfRotations %d' % NUMBER_OF_ROTATIONS,
        P + '  snap FALSE',
        P + '}',
    ]


def roller_lines(i, which, indent, colour=(0.10, 0.10, 0.11), face="B", damping=ROLLER_DAMPING,
                 suffix=True, brake=False):
    """A passive roller on the tail ('T') or nose ('N') block: face "B" =
    bottom (-z), axle local y, rolls along x; face "S" = the -y side, axle
    local z (the engine's Cylinder is Z-aligned, so the side roller needs no
    rotation and the bottom one is turned onto y with Rx(90 deg)). DEF is
    CELL_i_ROLL_<T|N><B|S> (v3) or CELL_i_ROLL_<T|N> (v2, suffix=False)."""
    d = def_name(i, "ROLL_%s%s" % (which, face if suffix else ""))
    P = " " * indent
    if face == "B":
        axis, at, rot = "0 1 0", (0.0, 0.0, ROLLER_Z), "rotation 1 0 0 1.5708 "
        inertia = (ROLLER_I_TRANS, ROLLER_I_AXIAL, ROLLER_I_TRANS)
    else:
        axis, at, rot = "0 0 1", (0.0, -ROLLER_Y, 0.0), ""
        inertia = (ROLLER_I_TRANS, ROLLER_I_TRANS, ROLLER_I_AXIAL)
    return [
        P + 'DEF %s HingeJoint {' % d,
        P + '  jointParameters HingeJointParameters {',
        P + '    axis %s' % axis,
        P + '    anchor %.4f %.4f %.4f' % at,
        P + '    dampingConstant %g' % damping,
        P + '  }',
    ] + ([
        P + '  device [ RotationalMotor { name "roller_%s%s_brake" maxTorque %g maxVelocity 200 } ]'
        % (which.lower(), face.lower(), ROLLER_BRAKE_TORQUE),
    ] if brake else []) + [
        P + '  endPoint DEF %s_SOLID Solid {' % d,
        P + '    translation %.4f %.4f %.4f' % at,
        P + '    name "cell_%d_roller_%s%s"' % (i, which.lower(), face.lower() if suffix else ""),
        P + '    children [',
        P + '      Pose { %schildren [ Shape { appearance %s '
            'geometry Cylinder { radius %.4f height %.4f } } ] }'
        % (rot, pbr(colour, 0.5, 0.2), ROLLER_RADIUS, ROLLER_HEIGHT),
        P + '    ]',
        P + '    boundingObject Pose { %schildren [ Cylinder { radius %.4f height %.4f } ] }'
        % (rot, ROLLER_RADIUS, ROLLER_HEIGHT),
        P + '    physics Physics {',
        P + '      density -1',
        P + '      mass %.4f' % ROLLER_MASS,
        P + '      centerOfMass [ 0 0 0 ]',
        P + '      inertiaMatrix [ %.4g %.4g %.4g, 0 0 0 ]' % inertia,
        P + '    }',
        P + '  }',
        P + '}',
    ]


def roller_set(i, which, indent, rollers):
    """rollers: False (v1), True / "v2" (bottom pair), "v3" (bottom + side)."""
    if not rollers:
        return []
    if rollers == "v3":
        return (roller_lines(i, which, indent, face="B", brake=True) +
                roller_lines(i, which, indent, face="S", brake=True))
    return roller_lines(i, which, indent, face="B", suffix=False)


def bodies_per_cell(rollers):
    return 6 if rollers == "v3" else (4 if rollers else 2)


def cell_vrml(i, pos, yaw, roll=0.0, colour=DEFAULT_COLOUR, ring=RING_FULL,
              rotation=None, rollers=False):
    """One cell as VRML lines: the Pose (translation + yaw) wrapping the
    `DEF CELL_i Robot` (roll). `rotation`, if given, is a full axis-angle
    for the Robot node and overrides roll (branch_placement's axis_angle).

    Pose-wrapping is load-bearing (alife P3): a top-level Solid teleport
    knocks ~51% off every other body's joint velocity in one tick; a
    Pose-wrapped one does nothing measurable."""
    i = int(i)
    x, y = float(pos[0]), float(pos[1])
    z = float(pos[2]) if len(pos) > 2 else spawn_z(rollers)
    max_torque = MAX_TORQUE_V2 if rollers else MAX_TORQUE
    if rotation is None:
        rotation = (1.0, 0.0, 0.0, float(roll))
    L = []
    A = L.append
    A('Pose {')
    A('  translation %.5f %.5f %.5f' % (x, y, z))
    A('  rotation 0 0 1 %.6f' % float(yaw))
    A('  children [')
    A('    DEF %s Robot {' % def_name(i))
    A('      rotation %.6f %.6f %.6f %.6f' % tuple(rotation))
    A('      name "cell_%d"' % i)
    A('      controller "<none>"')
    A('      customData ""')
    A('      children [')
    A('        Shape { appearance %s geometry Box { size %.3f %.3f %.3f } }'
      % (pbr(colour), BLOCK, BLOCK, BLOCK))
    # charge ring: visual only, on top of the tail block
    A('        Pose { translation 0 0 %.4f children [' % RING_Z)
    A('          DEF %s Shape {' % def_name(i, "RING"))
    A('            appearance DEF %s PBRAppearance {' % def_name(i, "RING_APP"))
    A('              baseColor 0.05 0.05 0.05')
    A('              emissiveColor %.3f %.3f %.3f' % tuple(ring))
    A('              emissiveIntensity 2')
    A('              roughness 0.4')
    A('              metalness 0')
    A('            }')
    A('            geometry Cylinder { height %.4f radius %.4f }' % (RING_HEIGHT, RING_RADIUS))
    A('          }')
    A('        ] }')
    L += connector_lines(i, "f_tail", 8)
    L += roller_set(i, "T", 8, rollers)
    A('        DEF %s HingeJoint {' % def_name(i, "HINGE"))
    A('          jointParameters DEF %s HingeJointParameters {' % def_name(i, "HINGE_PARAMS"))
    A('            axis 0 1 0')
    A('            anchor %.4f 0 0' % HINGE_X)
    A('            position 0')
    A('            minStop %.2f' % -HINGE_STOP)
    A('            maxStop %.2f' % HINGE_STOP)
    A('          }')
    A('          device [')
    # Without min/maxPosition the joint is a ke=0 velocity wheel and every
    # position target is silently ignored (alife README).
    A('            DEF %s RotationalMotor {' % def_name(i, "MOTOR"))
    A('              name "hinge"')
    A('              minPosition %.2f' % HINGE_MIN)
    A('              maxPosition %.2f' % HINGE_MAX)
    A('              maxTorque %.2f' % max_torque)
    A('              maxVelocity %.2f' % MAX_VELOCITY)
    A('            }')
    A('            PositionSensor { name "hinge_sensor" }')
    A('          ]')
    A('          endPoint DEF %s Solid {' % def_name(i, "NOSE"))
    A('            translation %.4f 0 0' % NOSE_X)
    A('            name "cell_%d_nose"' % i)
    A('            children [')
    A('              Shape { appearance %s geometry Box { size %.3f %.3f %.3f } }'
      % (pbr(colour), BLOCK, BLOCK, BLOCK))
    L += connector_lines(i, "f_nose", 14)
    L += connector_lines(i, "f_left", 14)
    L += connector_lines(i, "f_right", 14)
    L += roller_set(i, "N", 14, rollers)
    A('            ]')
    A('            boundingObject Box { size %.3f %.3f %.3f }' % (BLOCK, BLOCK, BLOCK))
    A('            physics Physics {')
    A('              density -1')
    A('              mass %.3f' % MASS)
    A('              centerOfMass [ 0 0 0 ]')
    A('              inertiaMatrix [ %.4g %.4g %.4g, 0 0 0 ]' % (INERTIA, INERTIA, INERTIA))
    A('            }')
    A('          }')
    A('        }')
    A('      ]')
    A('      boundingObject Box { size %.3f %.3f %.3f }' % (BLOCK, BLOCK, BLOCK))
    # Explicit tensor: the geometry-derived path excludes OmRobot and falls
    # back to a Husky preset (alife README); inertiaMatrix without
    # centerOfMass is a parse WARNING that drops the node to mode INVALID.
    A('      physics Physics {')
    A('        density -1')
    A('        mass %.3f' % MASS)
    A('        centerOfMass [ 0 0 0 ]')
    A('        inertiaMatrix [ %.4g %.4g %.4g, 0 0 0 ]' % (INERTIA, INERTIA, INERTIA))
    A('      }')
    A('    }')
    A('  ]')
    A('}')
    return L
