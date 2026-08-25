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

"""universal_cam -- DEPTH-ONLY pick-point camera for universal bin picking.

The whole point: NO registry, NO CAD, NO recognition, NO per-object model.
A top-down RangeFinder looks into the bin and this controller finds where a
suction cup could seal, from the depth image alone:

  score = highest surface  x  locally flat  x  widest solid patch

which is the classic suction pick-point heuristic (cf. Dex-Net 3.0 /
SuctionNet's seal models, here in its simplest honest form). It also
DETECTS THE BIN ITSELF from the same image (wall-top ring -> centroid +
min-area-rect yaw), so the bin can be dragged or rotated mid-demo and the
picker follows. It publishes:

  "t=<tick>;m=<material_px>;b=cx,cy,yaw;x,y,z;x,y,z;..."

- t: frame counter (the supervisor waits for it to advance after parking
  the arm out of view -- a stale frame is the arm, not the pile).
- m: count of in-bin pixels in the part-height band -- m ~ 0 means empty.
- b: detected bin pose (world centre + yaw); omitted if no ring in view.
- anchors: up to 3, best first, non-overlapping (>= 7 px apart).

World-frame pose estimates carry +-3 mm noise (ANYPICK_SEED seeds it) so
the supervisor cannot lean on sim-perfect depth.

Depth-convention (ray vs planar) is self-calibrated against a pixel that
views the known-empty stage floor, same trick as anypick_cam.
"""

import math
import os
import random

from omnisim import Robot

robot = Robot()
dt = int(robot.getBasicTimeStep())

# The depth pipeline is this demo's dominant per-tick cost: a 160x160 RangeFinder
# render, a 25,600-value readback, and a pure-Python find_bin + find_anchors (the
# latter sweeping rotations in 2-deg steps) all ran EVERY 16 ms basic step --
# 62.5 frames per simulated second. The pick controller consumes anchors a
# handful of times per pick and only ever waits for the 't=' counter to advance
# by 3, so almost all of that was thrown away. Sampling every CAM_EVERY steps
# cuts the render, the readback, the Python pass AND this controller's IPC
# round trip by the same factor. At the default 8 a fresh frame still lands
# every 128 ms, so the observe() wait for 3 frames costs 0.38 s of the 4 s it
# is allowed.
CAM_EVERY = max(1, int(os.environ.get("UNIVERSAL_CAM_EVERY", "8")))
CAM_DT = dt * CAM_EVERY

rf = robot.getDevice("bin_depth")
rf.enable(CAM_DT)

rng = random.Random(int(os.environ.get("ANYPICK_SEED", "0")) or None)
NOISE = 0.003

RF_W = RF_H = 160
RF_FOV = 0.6
_f = (RF_W / 2.0) / math.tan(RF_FOV / 2.0)
CAM_P = (0.46, 0.0, 1.15)               # must match the .wbt BIN_CAM pose
GROUND_Z = 0.0                          # stage floor (calibration reference)
BIN_HALF = 0.165                        # interior half-extent for MATERIAL count
ANCHOR_HALF = 0.160                     # slightly tighter half-extent for
                                        # ANCHORS: keeps them off the wall
                                        # lines despite ~cm centre error, but
                                        # wide enough that wall-adjacent parts
                                        # stay pickable (0.150 stranded 4/15
                                        # against the walls of a yawed bin)
FLOOR_TOP = 0.022                       # bin floor top face
MIN_PART_Z = FLOOR_TOP + 0.012          # below this = floor, not material
                                        # (0.012: catches a 16 mm slab lying
                                        # flat, stays above +-3 mm depth noise)
MAX_PART_Z = 0.14                       # above this = bin WALL, not material
WALL_ZLO, WALL_ZHI = 0.21, 0.28         # wall-TOP ring band (top ~0.241). Kept
                                        # tight: a wider band admits the upper
                                        # inner SIDE faces of the far walls
                                        # (parallax) and biased the centroid
                                        # ~3.5 cm (measured on the yawed bin)

_depth_is_ray = None


def _pixel_world(img, i, j):
    """World (x, y, z) of depth pixel (i=column, j=row)."""
    d = img[j * RF_W + i]
    if d is None or d != d or d <= 0.0 or d > 5.0:
        return None
    vy = (RF_W / 2.0 - i) / _f
    vz = (RF_H / 2.0 - j) / _f
    n = math.sqrt(1.0 + vy * vy + vz * vz)
    if _depth_is_ray:
        dx, dy, dz = d / n, d * vy / n, d * vz / n
    else:
        dx, dy, dz = d, d * vy, d * vz
    return (CAM_P[0] + dz, CAM_P[1] + dy, CAM_P[2] - dx)


def _calibrate(img):
    """Ray-vs-planar: which convention puts a far off-centre pixel (over the
    empty stage floor north of the bin) at z=GROUND_Z."""
    global _depth_is_ray
    i, j = RF_W // 2, 6                          # views floor at x ~ 0.80
    errs = []
    for is_ray in (True, False):
        d = img[j * RF_W + i]
        vz = (RF_H / 2.0 - j) / _f
        n = math.sqrt(1.0 + vz * vz)
        dx = d / n if is_ray else d
        errs.append(abs((CAM_P[2] - dx) - GROUND_Z))
    _depth_is_ray = errs[0] < errs[1]
    print("[universal_cam] depth convention: %s (err ray=%.3f planar=%.3f)"
          % ("RAY" if _depth_is_ray else "PLANAR", errs[0], errs[1]), flush=True)


def _world_to_pixel(x, y):
    dz_c = x - CAM_P[0]
    dy_c = y - CAM_P[1]
    depth = CAM_P[2] - FLOOR_TOP
    i = int(RF_W / 2.0 - (dy_c / depth) * _f)
    j = int(RF_H / 2.0 - (dz_c / depth) * _f)
    return max(2, min(RF_W - 3, i)), max(2, min(RF_H - 3, j))


def find_bin(img):
    """Detect the BIN POSE (cx, cy, yaw) from the depth image alone: the
    wall-top ring is the only material in the WALL_Z band, so its pixels
    trace the bin rectangle. Centroid gives the centre; a min-area-rectangle
    rotation search (2-deg steps, square-symmetric so 0..90 deg) gives the
    yaw. This is what lets the user DRAG or ROTATE the bin mid-demo: the
    picker re-detects it at every observation. Returns None if the ring is
    not in view."""
    pts = []
    for j in range(2, RF_H - 2, 2):
        for i in range(2, RF_W - 2, 2):
            p = _pixel_world(img, i, j)
            if p is not None and WALL_ZLO <= p[2] <= WALL_ZHI:
                pts.append((p[0], p[1]))
    if len(pts) < 40:
        return None
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    best_area, best_yaw = 1e9, 0.0
    best_mid = (0.0, 0.0)
    for deg in range(0, 90, 2):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        us = [(p[0] - cx) * c + (p[1] - cy) * s for p in pts]
        vs = [-(p[0] - cx) * s + (p[1] - cy) * c for p in pts]
        area = (max(us) - min(us)) * (max(vs) - min(vs))
        if area < best_area:
            best_area, best_yaw = area, th
            best_mid = ((max(us) + min(us)) / 2.0, (max(vs) + min(vs)) / 2.0)
    # Centre = MIDPOINT of the min-area rect, NOT the pixel centroid: the
    # far walls' inner faces leak extra pixels into the band on one side
    # (parallax), and the density-weighted centroid was ~4.6 cm off on an
    # off-axis bin -- enough to strand wall-adjacent parts outside the
    # anchor window. The rect midpoint only depends on the wall EXTENTS.
    mc, ms = math.cos(best_yaw), math.sin(best_yaw)
    cx2 = cx + best_mid[0] * mc - best_mid[1] * ms
    cy2 = cy + best_mid[0] * ms + best_mid[1] * mc
    return cx2, cy2, best_yaw


def find_anchors(img, bin_pose, k=3):
    """Top-k suction anchors over the DETECTED bin interior + material count.

    1. every 2nd pixel whose world point is inside the DETECTED bin frame
       (rotate by -yaw about the detected centre) and in the part-height
       band is material -- wall tops (z > MAX_PART_Z) are excluded so the
       rim can never become an anchor;
    2. candidates in the top height band (<= 12 mm below the pile top) are
       kept only when locally FLAT (3x3 spread < 8 mm -- rejects steep
       flanks, accepts the gentle crown of a lying cylinder or a sphere);
    3. rank by SOLID COVERAGE (same-height pixels in a pad-sized window):
       the widest patch is where a cup actually seals, not a limb edge;
    4. greedily take k anchors >= 7 px apart.
    """
    bx, by, byaw = bin_pose
    c, s = math.cos(byaw), math.sin(byaw)
    mat = []
    for j in range(2, RF_H - 2, 2):
        for i in range(2, RF_W - 2, 2):
            p = _pixel_world(img, i, j)
            if p is None or p[2] < MIN_PART_Z or p[2] > MAX_PART_Z:
                continue
            u = (p[0] - bx) * c + (p[1] - by) * s
            v = -(p[0] - bx) * s + (p[1] - by) * c
            if abs(u) > BIN_HALF or abs(v) > BIN_HALF:
                continue
            mat.append((p[2], i, j, p))
    if not mat:
        return 0, []
    mat.sort(reverse=True)
    ztop = mat[0][0]

    def _scored(band, flat):
        scored = []
        for z, i, j, p in mat:
            if ztop - z > band:
                break
            zs = []
            ok = True
            for dj in (-2, 0, 2):
                for di in (-2, 0, 2):
                    q = _pixel_world(img, i + di, j + dj)
                    if q is None:
                        ok = False
                        break
                    zs.append(q[2])
                if not ok:
                    break
            if not ok or max(zs) - min(zs) >= flat:
                continue
            cover = 0
            for dj in range(-4, 5, 2):
                for di in range(-4, 5, 2):
                    q = _pixel_world(img, i + di, j + dj)
                    if q is not None and abs(q[2] - z) < 0.006:
                        cover += 1
            scored.append((cover, z, i, j, p))
        scored.sort(reverse=True)
        return scored

    # SEARCH LADDER: prefer the strict top-band/flatness pass, but never
    # return nothing while material remains -- low or narrow objects (a
    # 16 mm slab, a 24 mm bracket limb ~5 px wide) fail the strict gates
    # after the tall stuff is gone, and "no anchor" would strand them.
    scored = []
    for band, flat in ((0.012, 0.008), (0.030, 0.008), (0.060, 0.013),
                       (9.9, 0.013)):
        scored = _scored(band, flat)
        if scored:
            break
    if not scored:
        # No anchor at any ladder rung -> report none. There is NO
        # flatness-exempt fallback on purpose: "highest material pixel"
        # happily anchors on the bin's inner WALL FACE (a vertical surface
        # slips under MAX_PART_Z near its base) and sent the arm pressing
        # the wall in a loop (measured on the yawed-bin test). If the last
        # rung (any height band, 13 mm flatness) finds nothing, whatever
        # material remains is not suction-graspable from above.
        return len(mat), []
    out = []
    for cover, z, i, j, p in scored:
        u = (p[0] - bx) * c + (p[1] - by) * s
        v = -(p[0] - bx) * s + (p[1] - by) * c
        if abs(u) > ANCHOR_HALF or abs(v) > ANCHOR_HALF:
            continue                             # too close to a wall line
        if any(abs(i - oi) < 7 and abs(j - oj) < 7 for oi, oj, _ in out):
            continue
        out.append((i, j, p))
        if len(out) >= k:
            break
    return len(mat), [p for _i, _j, p in out]


_tick = 0
while robot.step(CAM_DT) != -1:
    _tick += 1
    img = rf.getRangeImage()
    if img is None:
        continue
    if _depth_is_ray is None:
        try:
            _calibrate(img)
        except Exception:
            _depth_is_ray = True
    try:
        bp = find_bin(img)
    except Exception:
        bp = None
    if bp is None:
        robot.setCustomData("t=%d;m=0" % _tick)
        continue
    try:
        m, anchors = find_anchors(img, bp)
    except Exception:
        m, anchors = 0, []
    toks = ["t=%d" % _tick, "m=%d" % m,
            "b=%.4f,%.4f,%.4f" % (bp[0] + rng.uniform(-NOISE, NOISE),
                                  bp[1] + rng.uniform(-NOISE, NOISE),
                                  bp[2])]
    for a in anchors:
        toks.append("%.4f,%.4f,%.4f"
                    % (a[0] + rng.uniform(-NOISE, NOISE),
                       a[1] + rng.uniform(-NOISE, NOISE),
                       a[2] + rng.uniform(-NOISE, NOISE)))
    robot.setCustomData(";".join(toks))
