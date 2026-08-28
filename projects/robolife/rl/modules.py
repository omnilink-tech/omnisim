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

"""RoboLife module catalogue + VRML emitter (DESIGN.md "Modules"). Owner: A.

Shared by B (docking geometry reads `PLUG_OFFSET`, `plug_pose`) and C (the
energy rules read `CATALOGUE[type]["effect"]`, the supervisor reads mass).

Frames (measured against the expanded Husky, see robots/husky_base.txt):

  * A module Solid's origin is the CENTRE OF ITS FOOTPRINT ON THE FLOOR: the
    collider is lifted by half its height, so a loose module is authored at
    z = 0 (+ a few mm) and rests there. That keeps every type's plug at the
    same height, whatever the box height.
  * The passive `Connector name "plug"` sits on the module's -x face at
    local (-length/2, 0, PLUG_Z) with `rotation 0 0 1 pi`, so its normal
    (a Connector's local +x) points along the module's -x. A robot whose
    front socket (normal +x, world z 0.25) drives up to that face is
    x-anti-parallel to it, which is what `OmConnector::isXAlignedWith`
    requires; the z axes stay parallel, which satisfies the rotational check.
  * PLUG_Z = 0.25 is a WORLD height: the Husky origin sits 0.13228 m above
    the ground, so the socket is authored at robot-frame z 0.11772 to meet
    it. DESIGN.md quotes 0.25 for both; the robot-frame number is the only
    deviation, and it is the one that makes the two meet.

Docked pose: robot yaw == module yaw and the robot origin is
`plug_world - R(yaw) @ (SOCKET_X, 0)`, i.e. `module_origin + R(yaw) @
(-(length/2 + SOCKET_X), 0)` -- see `docked_robot_xy()`.
"""
import math

# Robot-side constants B and C need without importing the world template.
SOCKET_X = 0.55          # front socket x in the robot frame (rear: -0.55)
PLUG_Z = 0.25            # world height of every plug / socket
HUSKY_ORIGIN_Z = 0.13228  # Husky origin above the ground (URDF base_footprint)
SOCKET_Z_ROBOT = PLUG_Z - HUSKY_ORIGIN_Z   # 0.11772, authored in husky_base.txt

# Same tolerances as the sockets (DESIGN.md).
DISTANCE_TOLERANCE = 0.08
AXIS_TOLERANCE = 0.45
ROTATION_TOLERANCE = 0.6
NUMBER_OF_ROTATIONS = 4

# type -> geometry, mass, colour, effect constants (DESIGN.md table).
# "size" is (x, y, z) for boxes; the cylinder is (radius, height).
CATALOGUE = {
    "battery": {
        "shape": "box", "size": (0.34, 0.30, 0.24), "mass": 6.0,
        "colour": (0.05, 0.35, 0.12),           # dark green
        "effect": {"capacity_mult": 1.5, "idle_w": 0.5},
    },
    "solar": {
        "shape": "box", "size": (0.40, 0.40, 0.06), "mass": 2.5,
        "colour": (0.12, 0.30, 0.85),           # blue
        "post": (0.03, 0.16),                   # visual post (radius, height) under the panel
        "effect": {"solar_w": 6.0},
    },
    "mast": {
        "shape": "cylinder", "size": (0.05, 0.60), "mass": 1.5,
        "colour": (0.95, 0.50, 0.08),           # orange
        "effect": {"detect_range_m": 12.0},     # default detection range is 6 m
    },
    "armor": {
        "shape": "box", "size": (0.36, 0.30, 0.20), "mass": 5.0,
        "colour": (0.45, 0.46, 0.48),           # grey
        "effect": {"impact_mult": 2.0},
    },
}
TYPES = tuple(CATALOGUE)
DEFAULT_DETECT_RANGE_M = 6.0


def half_length(mtype):
    """Half of the module's extent along its own x (the docking axis)."""
    m = CATALOGUE[mtype]
    return (m["size"][0] if m["shape"] == "box" else m["size"][0] * 2.0) / 2.0


def height(mtype):
    m = CATALOGUE[mtype]
    h = m["size"][2] if m["shape"] == "box" else m["size"][1]
    if "post" in m:
        h += m["post"][1]
    return h


def mass(mtype):
    return float(CATALOGUE[mtype]["mass"])


def plug_offset(mtype):
    """Plug position in the module frame: (-length/2, 0, PLUG_Z)."""
    return (-half_length(mtype), 0.0, PLUG_Z)


def plug_pose(mtype, pos, yaw):
    """World (x, y, z, yaw_of_normal) of the plug of a module at pos/yaw.
    The normal points along the module's -x, i.e. yaw + pi."""
    dx, dy, dz = plug_offset(mtype)
    c, s = math.cos(yaw), math.sin(yaw)
    return (pos[0] + c * dx - s * dy, pos[1] + s * dx + c * dy,
            (pos[2] if len(pos) > 2 else 0.0) + dz, yaw + math.pi)


def docked_robot_xy(mtype, pos, yaw):
    """Where the robot origin is when its FRONT socket is mated to this
    module's plug: yaw equal to the module's, origin at
    module_origin + R(yaw) @ (-(half_length + SOCKET_X), 0)."""
    d = -(half_length(mtype) + SOCKET_X)
    return (pos[0] + math.cos(yaw) * d, pos[1] + math.sin(yaw) * d)


def _pbr(rgb, rough=0.6, metal=0.1):
    return ('PBRAppearance { baseColor %.3f %.3f %.3f roughness %.2f metalness %.2f }'
            % (rgb[0], rgb[1], rgb[2], rough, metal))


def module_vrml(j, mtype, pos, yaw=0.0, indent=0):
    """`DEF MODULE_<j> Solid` lines: physics, collider, colour, and the
    passive plug Connector. Origin at the footprint centre (see the module
    docstring); `pos` is (x, y[, z]) with z defaulting to 0.01 so a loose
    module settles onto the floor instead of starting inside it."""
    m = CATALOGUE[mtype]
    z = pos[2] if len(pos) > 2 else 0.01
    col = m["colour"]
    L = []
    A = L.append
    A('DEF MODULE_%d Solid {' % j)
    A('  translation %.4f %.4f %.4f' % (pos[0], pos[1], z))
    A('  rotation 0 0 1 %.5f' % yaw)
    A('  name "module_%d"' % j)
    A('  model "%s"' % mtype)
    A('  children [')
    if m["shape"] == "box":
        sx, sy, sz = m["size"]
        zc = sz / 2.0
        if "post" in m:
            pr, ph = m["post"]
            zc = ph + sz / 2.0
            A('    Pose { translation 0 0 %.4f children [ Shape { appearance %s '
              'geometry Cylinder { radius %.3f height %.3f } } ] }'
              % (ph / 2.0, _pbr((0.2, 0.2, 0.22)), pr, ph))
        A('    Pose {')
        A('      translation 0 0 %.4f' % zc)
        A('      children [ Shape { appearance %s geometry Box { size %.3f %.3f %.3f } } ]'
          % (_pbr(col), sx, sy, sz))
        A('    }')
        collider = ('Pose { translation 0 0 %.4f children [ Box { size %.3f %.3f %.3f } ] }'
                    % (zc, sx, sy, sz))
        ixx = m["mass"] / 12.0 * (sy * sy + sz * sz)
        iyy = m["mass"] / 12.0 * (sx * sx + sz * sz)
        izz = m["mass"] / 12.0 * (sx * sx + sy * sy)
        com = (0.0, 0.0, zc)
    else:
        r, h = m["size"]
        zc = h / 2.0
        A('    Pose {')
        A('      translation 0 0 %.4f' % zc)
        A('      children [ Shape { appearance %s geometry Cylinder { radius %.3f height %.3f } } ]'
          % (_pbr(col), r, h))
        A('    }')
        collider = ('Pose { translation 0 0 %.4f children [ Cylinder { radius %.3f height %.3f } ] }'
                    % (zc, r, h))
        ixx = iyy = m["mass"] / 12.0 * (3 * r * r + h * h)
        izz = m["mass"] / 2.0 * r * r
        com = (0.0, 0.0, zc)
    # A visual plug: a short stub on the -x face at plug height so the mate
    # point is readable in a frame. Visual only (no collider).
    px, py, pz = plug_offset(mtype)
    A('    Pose { translation %.4f 0 %.4f rotation 0 1 0 1.5708 children [ Shape { appearance %s '
      'geometry Cylinder { radius 0.03 height 0.04 } } ] }'
      % (px + 0.02, pz, _pbr((0.9, 0.85, 0.2), 0.4, 0.6)))
    A('    DEF MODULE_%d_PLUG Connector {' % j)
    A('      name "plug"')
    A('      translation %.4f %.4f %.4f' % (px, py, pz))
    A('      rotation 0 0 1 3.14159')
    A('      type "passive"')
    A('      distanceTolerance %g' % DISTANCE_TOLERANCE)
    A('      axisTolerance %g' % AXIS_TOLERANCE)
    A('      rotationTolerance %g' % ROTATION_TOLERANCE)
    A('      numberOfRotations %d' % NUMBER_OF_ROTATIONS)
    A('      snap FALSE')
    A('      boundingObject NULL')
    A('      physics NULL')
    A('    }')
    A('  ]')
    A('  boundingObject %s' % collider)
    A('  physics Physics {')
    A('    density -1')
    A('    mass %.3f' % m["mass"])
    A('    centerOfMass [ %.4f %.4f %.4f ]' % com)
    A('    inertiaMatrix [ %.6g %.6g %.6g, 0 0 0 ]' % (ixx, iyy, izz))
    A('  }')
    A('}')
    P = ' ' * indent
    return [P + ln for ln in L]
