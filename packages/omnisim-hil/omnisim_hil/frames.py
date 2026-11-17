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

"""The one place OmniSim's frames become MAVLink's frames.

OmniSim is **ENU / FLU**: the world is X-east, Y-north, Z-up, and a robot is
X-forward, Y-left, Z-up. MAVLink is **NED / FRD**: the world is X-north,
Y-east, Z-down, and the body is X-forward, Y-right, Z-down.

There are TWO conversions here, not one, and they are different matrices. This
module used to claim they were the same involution; they are not, and the
difference is the whole reason the module exists.

    world:  M = [[0, 1, 0],   ENU -> NED: swap east/north, negate up.
                 [1, 0, 0],
                 [0, 0, -1]]

    body:   D = diag(1, -1, -1)   FLU -> FRD: keep forward, negate left and up.

Both are involutions (``M M == I``, ``D D == I``) and both are proper rotations
(determinant +1), so each converts in both directions. But applying ``M`` to a
body vector maps the nose onto the right wing: ``M (1,0,0) = (0,1,0)``. An
aircraft accelerating forward then reports that acceleration as lateral, and the
attitude of a machine flying east reports as a heading of north.

That is exactly the failure mode this module was written to prevent: a sign or
axis error here does not crash anything, it produces an autopilot flying a
rotated, mirrored aircraft, and it is invisible until something is upside down.
``tests/test_frames.py`` now pins both maps against named physical cases rather
than only against the involution property, which the wrong matrix also satisfies.

An attitude matrix converts as ``M R D``: the right factor re-expresses the body
basis, the left factor the world basis. With one matrix on both sides -- the old
``M R M`` -- the body half is wrong in exactly the way above.
"""

from __future__ import annotations

import math
from typing import Tuple

from .vec3 import Mat3, Vec3, euler_from_mat, quat_from_mat


def swap_ned_enu(v: Vec3) -> Vec3:
    """ENU <-> NED for a WORLD vector. Its own inverse.

    ``(east, north, up) -> (north, east, down)``. Do not reach for this with a
    body vector -- see :func:`swap_flu_frd`, which is a different matrix.
    """
    return (v[1], v[0], -v[2])


def swap_flu_frd(v: Vec3) -> Vec3:
    """FLU <-> FRD for a BODY vector. Its own inverse.

    ``(forward, left, up) -> (forward, right, down)``. Forward is common to both
    conventions and must survive untouched; only the lateral and vertical axes
    flip.
    """
    return (v[0], -v[1], -v[2])


#: Readability aliases. The name records which frame pair the call site meant,
#: which is what a reader needs. World and body are deliberately NOT the same
#: function: aliasing them is the defect this module now tests against.
enu_to_ned = swap_ned_enu
ned_to_enu = swap_ned_enu
flu_to_frd = swap_flu_frd
frd_to_flu = swap_flu_frd


def rotation_flu_enu_to_frd_ned(m: Mat3) -> Mat3:
    """Convert a body-to-world rotation from FLU-in-ENU to FRD-in-NED.

    ``M R D``: the right factor re-expresses the body basis (FLU -> FRD), the
    left factor the world basis (ENU -> NED). The two factors are different
    matrices, and using one for both silently yaws the reported attitude by
    90 degrees and mirrors it.
    """
    # M R swaps rows 0 and 1 and negates row 2; right-multiplying by
    # D = diag(1,-1,-1) then negates columns 1 and 2. Written out rather than
    # looped because this runs every control tick and the index pattern is
    # easier to check by eye than a triple loop.
    return (m[3], -m[4], -m[5],
            m[0], -m[1], -m[2],
            -m[6], m[7], m[8])


def quaternion_frd_ned(m: Mat3) -> Tuple[float, float, float, float]:
    """(w, x, y, z) attitude for MAVLink, from an OmniSim orientation matrix."""
    return quat_from_mat(rotation_flu_enu_to_frd_ned(m))


def euler_frd_ned(m: Mat3) -> Vec3:
    """(roll, pitch, yaw) in the aerospace sense, from an OmniSim matrix.

    Yaw is measured from NORTH, increasing toward EAST -- a compass heading in
    radians, not the counter-clockwise-from-east angle OmniSim reports.
    """
    return euler_from_mat(rotation_flu_enu_to_frd_ned(m))


#: WGS-84 mean radius. Adequate for a local-tangent-plane approximation over
#: the tens of kilometres a delivery aircraft flies; this is deliberately NOT a
#: full geodetic conversion, and the docstring below says where it stops being
#: honest.
EARTH_RADIUS_M = 6371000.0


def local_to_global(
    east_m: float,
    north_m: float,
    up_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float = 0.0,
) -> Tuple[float, float, float]:
    """Flat-earth local ENU metres to (lat_deg, lon_deg, alt_m).

    A tangent-plane approximation about the origin. Error grows with the square
    of the distance from it: under a metre within 10 km, a few metres by 50 km.
    That is fine for placing an aircraft in a GPS message and wrong for
    anything that needs survey accuracy, so do not reuse this for geodesy.
    """
    lat = origin_lat_deg + math.degrees(north_m / EARTH_RADIUS_M)
    coslat = math.cos(math.radians(origin_lat_deg))
    # Guard the poles: cos(lat) -> 0 makes the longitude scale diverge, and a
    # simulated aircraft parked at 90 degrees would produce infinities in the
    # sensor stream rather than an obvious refusal.
    if abs(coslat) < 1e-9:
        coslat = 1e-9
    lon = origin_lon_deg + math.degrees(east_m / (EARTH_RADIUS_M * coslat))
    return (lat, lon, origin_alt_m + up_m)


def global_to_local(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float = 0.0,
) -> Tuple[float, float, float]:
    """Inverse of :func:`local_to_global`, returning ENU metres."""
    north = math.radians(lat_deg - origin_lat_deg) * EARTH_RADIUS_M
    coslat = math.cos(math.radians(origin_lat_deg))
    if abs(coslat) < 1e-9:
        coslat = 1e-9
    east = math.radians(lon_deg - origin_lon_deg) * EARTH_RADIUS_M * coslat
    return (east, north, alt_m - origin_alt_m)
