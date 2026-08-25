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

"""Geodetic <-> local-frame helpers for outdoor GPS navigation.

Pure Python, zero Webots dependency, so it can be unit-tested standalone
(``python geo.py`` runs the self-check at the bottom) and reused by any
Omni Quest controller or the OmniLink mission agent.

Frame convention (matches OmniSim R2025a and ROS REP-103):

    Local metric frame is **ENU**: +x = East, +y = North, +z = Up.
    A heading (yaw) of 0 means the robot faces **East**; yaw increases
    counter-clockwise (toward North).

We use the **equirectangular** flat-earth approximation for lat/lon <-> ENU.
Over the scale of a field or campus (< a few km) its error vs the full
WGS-84 ellipsoid is centimetre-level, and because the controller converts
the GPS *reading* back to ENU with the same model it uses to place the
waypoints, the round-trip is internally exact regardless of how the
simulator's GPS node computes WGS-84 forward. The full ellipsoid path
(``geodetic_to_enu_wgs84``) is provided for when that matters.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Mean Earth radius (m) — the standard value used by the equirectangular
# approximation and by ROS robot_localization's flat-earth fallback.
EARTH_RADIUS_M = 6_371_000.0

# WGS-84 ellipsoid constants (for the exact geodetic path).
_WGS84_A = 6_378_137.0              # semi-major axis (m)
_WGS84_E2 = 6.694_379_990_14e-3     # first eccentricity squared


# ---------------------------------------------------------------------------
# Small-angle helpers
# ---------------------------------------------------------------------------

def wrap_pi(angle: float) -> float:
    """Wrap an angle (rad) to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# Equirectangular (flat-earth) lat/lon <-> ENU. Fast, self-consistent for the
# place-waypoint / read-GPS round trip, and — because it uses the *local*
# radii of curvature rather than a single mean radius — accurate to cm vs the
# full ellipsoid over a campus-scale area.
# ---------------------------------------------------------------------------

def _local_radii(ref_lat_deg: float) -> tuple:
    """(meridian, prime-vertical) radii of curvature (m) at a latitude.

    M governs metres-per-radian of latitude (north); N governs
    metres-per-radian of longitude (east, scaled by cos(lat)).
    """
    lat0 = math.radians(ref_lat_deg)
    s2 = math.sin(lat0) ** 2
    denom = 1.0 - _WGS84_E2 * s2
    n = _WGS84_A / math.sqrt(denom)                    # prime vertical
    m = _WGS84_A * (1.0 - _WGS84_E2) / (denom ** 1.5)  # meridian
    return m, n


def geodetic_to_enu(lat_deg: float, lon_deg: float,
                    ref_lat_deg: float, ref_lon_deg: float,
                    alt_m: float = 0.0, ref_alt_m: float = 0.0) -> tuple:
    """(lat, lon, alt) -> local ENU (east_m, north_m, up_m) about a datum.

        E = N * cos(lat0) * (lon - lon0)
        N = M * (lat - lat0)
        U = alt - alt0

    with M, N the local meridian / prime-vertical radii. Angles in degrees,
    output in metres.
    """
    m, n = _local_radii(ref_lat_deg)
    lat0 = math.radians(ref_lat_deg)
    east = n * math.cos(lat0) * math.radians(lon_deg - ref_lon_deg)
    north = m * math.radians(lat_deg - ref_lat_deg)
    up = alt_m - ref_alt_m
    return east, north, up


def enu_to_geodetic(east_m: float, north_m: float,
                    ref_lat_deg: float, ref_lon_deg: float,
                    up_m: float = 0.0, ref_alt_m: float = 0.0) -> tuple:
    """Inverse of :func:`geodetic_to_enu` — local ENU -> (lat, lon, alt).

    Handy for *authoring* a route as metric offsets and emitting the
    lat/lon waypoints a real GPS mission would carry.
    """
    m, n = _local_radii(ref_lat_deg)
    lat0 = math.radians(ref_lat_deg)
    lat_deg = ref_lat_deg + math.degrees(north_m / m)
    lon_deg = ref_lon_deg + math.degrees(east_m / (n * math.cos(lat0)))
    return lat_deg, lon_deg, ref_alt_m + up_m


# ---------------------------------------------------------------------------
# Exact geodetic -> ECEF -> ENU (full WGS-84 ellipsoid). Use when the
# flat-earth error is unacceptable (large areas, high latitudes).
# ---------------------------------------------------------------------------

def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def geodetic_to_enu_wgs84(lat_deg: float, lon_deg: float, alt_m: float,
                          ref_lat_deg: float, ref_lon_deg: float,
                          ref_alt_m: float) -> tuple:
    """Exact (lat, lon, alt) -> ENU about a datum via the WGS-84 ellipsoid."""
    x, y, z = _geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = _geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)
    sin_lat0, cos_lat0 = math.sin(lat0), math.cos(lat0)
    sin_lon0, cos_lon0 = math.sin(lon0), math.cos(lon0)
    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


# ---------------------------------------------------------------------------
# Distance + bearing
# ---------------------------------------------------------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance (m) between two lat/lon points (degrees)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_enu(east_from: float, north_from: float,
                east_to: float, north_to: float) -> float:
    """Heading (rad, ENU: 0 = East, CCW +) from one ENU point toward another."""
    return math.atan2(north_to - north_from, east_to - east_from)


# ---------------------------------------------------------------------------
# Waypoint container
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Waypoint:
    """A GPS goal. ``name`` is for logging; ``lat``/``lon`` in degrees."""
    lat: float
    lon: float
    name: str = ""


# ---------------------------------------------------------------------------
# Standalone self-check: `python geo.py`
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    ref_lat, ref_lon = 40.67, -73.94          # matches the M1 world datum

    # 1) ENU round-trips through lat/lon exactly under the flat-earth model.
    for e, n in ((25.0, 0.0), (0.0, 40.0), (-12.5, 31.0), (100.0, -60.0)):
        lat, lon, _ = enu_to_geodetic(e, n, ref_lat, ref_lon)
        e2, n2, _ = geodetic_to_enu(lat, lon, ref_lat, ref_lon)
        assert abs(e - e2) < 1e-6 and abs(n - n2) < 1e-6, (e, n, e2, n2)

    # 2) Flat-earth vs full ellipsoid agree to < 5 cm at 100 m offset.
    lat, lon, _ = enu_to_geodetic(100.0, 100.0, ref_lat, ref_lon)
    ef, nf, _ = geodetic_to_enu(lat, lon, ref_lat, ref_lon)
    ew, nw, _ = geodetic_to_enu_wgs84(lat, lon, 0.0, ref_lat, ref_lon, 0.0)
    err = math.hypot(ef - ew, nf - nw)
    assert err < 0.05, f"flat-earth vs ellipsoid disagreement {err:.4f} m"

    # 3) haversine (mean-radius sphere) tracks the planar norm (local-radii
    #    ellipsoid) to within their ~0.06% model difference at short range.
    d_hav = haversine_m(ref_lat, ref_lon, lat, lon)
    d_planar = math.hypot(ef, nf)
    assert abs(d_hav - d_planar) < 0.2, (d_hav, d_planar)

    # 4) bearing sanity: due-East offset -> 0 rad, due-North -> pi/2.
    assert abs(bearing_enu(0, 0, 10, 0) - 0.0) < 1e-9
    assert abs(bearing_enu(0, 0, 0, 10) - math.pi / 2) < 1e-9

    print("geo.py self-check OK "
          f"(flat-vs-ellipsoid err @100m = {err * 100:.2f} cm)")
