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

"""OMNITUG500 autonomous explore-and-map controller.

A real sense -> map -> plan -> drive loop, all in pure Python:
  * SENSE  - read both corner Lidars, transform returns to the world frame using
             the rover's known pose.
  * MAP    - integrate the returns into a log-odds 2D occupancy grid (free along
             each ray, occupied at a real hit), classify into free/occ/unknown.
             The rover footprint is stamped free (it stands on free ground).
  * PLAN   - find frontiers (free cells next to unknown), CLUSTER them, pick the
             nearest sizeable cluster at least a body-length away, A* a path to it
             over free (obstacle-inflated) cells, and COMMIT until it is reached.
  * DRIVE  - pure-pursuit the path with a kinematic diff-drive model (bounded
             linear/angular speed, nonholonomic), moving the rover + scanners.
Stops when no reachable frontier remains.

LOCALISATION (SLAM, default ON in physics; --no-slam reverts): the controller
does NOT use the simulator's ground-truth pose for mapping/navigation. It
estimates pose from wheel odometry (true motion + realistic noise, which drifts)
and corrects that estimate by matching each lidar scan against the map built so
far (correlative scan-to-map matching, like Hector/Cartographer local SLAM). The
ground-truth pose is read ONLY to synthesise the odometry and to score drift.
This is what lets the same code transfer to the real robot.

Snapshots (grid + pose + path + frontier + trajectory) are written to
_scratch/omnitug500_explore/grid_*.json for offline visualisation.
"""

import heapq
import json
import math
import os
import random
import sys
import traceback
from collections import deque

_OUT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "..", "..", "_scratch", "omnitug500_explore"))


def _boot(msg):
    try:
        os.makedirs(_OUT, exist_ok=True)
        with open(os.path.join(_OUT, "debug.log"), "a", encoding="utf-8") as f:
            f.write(msg + "\n")
    except Exception:
        pass


_boot("import start")
try:
    from omnisim import Supervisor
    _boot("import OK")
except Exception:
    _boot("import FAILED:\n" + traceback.format_exc())
    raise

SCANNERS = {
    "scanner_front_right": (0.290, 0.537, 0.7854),
    "scanner_rear_left":  (-0.290, -0.536, 3.9270),
}

# --- occupancy grid -------------------------------------------------------
def _argf(flag, n):
    """Read n floats following `flag` in argv, else None."""
    if flag in sys.argv:
        i = sys.argv.index(flag)
        try:
            return [float(sys.argv[i + 1 + k]) for k in range(n)]
        except (IndexError, ValueError):
            return None
    return None


_b = _argf("--bounds", 4)              # --bounds X0 Y0 X1 Y1  (grid extent, m)
X0, Y0, X1, Y1 = _b if _b else (-6.6, -5.6, 6.6, 5.6)
_r = _argf("--res", 1)                  # --res RES  (cell size, m)
RES = _r[0] if _r else 0.15
NX = int(round((X1 - X0) / RES))
NY = int(round((Y1 - Y0) / RES))
VERIFY_CLEAR = "--verify-clearance" in sys.argv   # ground-truth clearance check
# Occupied evidence is "sticky": a detected obstacle accrues strong occupied
# log-odds and is NOT erased by a few grazing free rays (which happen when an
# obstacle slips into a corner scanner's blind wedge). This keeps obstacles in
# the map so both the planner and the collision guard keep avoiding them.
L_FREE, L_OCC, L_CLAMP = 0.25, 1.3, 6.0
T_FREE, T_OCC = -0.5, 0.45

# --- exploration / control -----------------------------------------------
RAY_STRIDE = 2
V_MAX = 0.75               # m/s
W_MAX = 1.6               # rad/s (gentler, more realistic turning)
LOOKAHEAD = 0.9           # m (larger = smoother, less twitchy steering)
STEER_KP = 1.2            # pure-pursuit heading gain (low enough to be stable
                          # under the 1-tick physics feedback lag)
DW_MAX = 0.30             # max change in commanded yaw-rate per tick (no jerk)
GOAL_TOL = 0.45
_inf = _argf("--inflate", 1)
# Inflation should be >= the rover's CIRCUMSCRIBED radius (sqrt(hw^2+hl^2) ~ 0.72 m
# for the 0.72 x 1.26 m chassis) so the whole footprint stays clear at any heading
# -- a half-width disc lets the long ends clip obstacles in turns.
INFLATE = _inf[0] if _inf else 0.65   # planner inflation (m); --inflate overrides
R_GUARD = INFLATE - 0.05              # collision-guard radius, just inside inflation
REPLAN_EVERY = 16          # ticks between PATH refreshes (target is kept)
SAFE_DIST = 0.45           # frontal reactive stop distance (centre -> hit)
SEED_STEPS = 8
DUMP_EVERY = 40
MAX_STEPS = 24000          # don't self-terminate before a full explore completes
# --- physics mode (force-controlled chassis so wall CONTACTS can stop it; a
#     per-tick setVelocity would override contacts and tunnel through walls) ---
PHYS_MASS = 40.0
PHYS_IZZ = 6.8             # ~ m/12*(w^2+l^2) for the 0.70x1.24 chassis
PHYS_KV = 10.0             # linear velocity-tracking gain
PHYS_KW = 12.0             # yaw velocity-tracking gain
PHYS_FMAX = 250.0          # N  (finite, so a wall normal force can overpower it)
PHYS_TMAX = 60.0           # N*m
RENDER_MAP = "--no-live-map" not in sys.argv   # live floor overlay (skip = faster)
MAP_EVERY = 60             # ticks between map-overlay rebuilds
_mds = _argf("--map-ds", 1)
MAP_DS = int(_mds[0]) if _mds else 2   # overlay downsample (coarser = lighter/faster)
MAP_FREE_Z = 0.02          # overlay heights (below the 0.131 scan plane)
MAP_OCC_Z = 0.05
MIN_FR_DIST = 0.5          # ignore frontiers closer than this (m)
MIN_CLUSTER = 3            # ignore frontier clusters smaller than this
ROVER_FREE_R = 0.42        # stamp this radius under the rover as free (m)
STALL_TICKS = 70           # no map growth for this long -> re-choose target
CHOOSE_RETRY = 10          # ticks between target-selection retries when idle
COMPLETE_AFTER = 250       # ticks with no target -> declare exploration complete
COMPLETE_HOLD = 60         # extra idle ticks after 'done' before quitting cleanly

# --- SLAM: localise from noisy odometry + lidar scan-matching (real-robot) ---
# Default ON in physics mode: the controller does NOT read the simulator's
# ground-truth pose for mapping/navigation -- it estimates pose from wheel
# odometry (true motion + realistic noise -> drift) and corrects that estimate by
# matching each lidar scan to the map it has built so far (correlative scan-to-map
# matching, like Hector/Cartographer local SLAM). The ground-truth pose is used
# ONLY to synthesise the odometry and to score drift afterwards. `--no-slam`
# reverts to the old known-pose mapping.
SLAM = "--no-slam" not in sys.argv
ODO_T = 0.025              # odometry translation noise (fraction of distance moved)
ODO_R = 0.025              # odometry rotation noise (fraction of rotation)
ODO_TR = 0.012             # heading drift induced per metre translated (rad/m)
SM_EVERY = 4               # ticks between scan-to-map corrections
SM_STRIDE = 6              # ray stride for the scan-matcher point cloud
SM_OCC_REFRESH = 24        # ticks between rebuilds of the matcher's likelihood field
SM_MIN_AVG = 0.6           # reject a correction if avg field weight per point is below this
ODO_SEED = 1234            # deterministic odometry-noise stream
# realistic 2D-lidar imperfection (applied to every range the controller reads, so
# SLAM + mapping see a real noisy sensor, not a perfect one). --clean-lidar opts out.
LIDAR_NOISE = 0.0 if "--clean-lidar" in sys.argv else 0.02   # range noise sigma (m)
LIDAR_DROPOUT = 0.0 if "--clean-lidar" in sys.argv else 0.015  # fraction of rays that drop
LIDAR_SEED = 777
# graded likelihood kernel: occupied cell scores highest, neighbours less, so the
# alignment objective has a gradient (-> sub-cell peak instead of a flat plateau).
SM_KERNEL = [(0, 0, 4), (1, 0, 3), (-1, 0, 3), (0, 1, 3), (0, -1, 3),
             (1, 1, 2), (1, -1, 2), (-1, 1, 2), (-1, -1, 2),
             (2, 0, 1), (-2, 0, 1), (0, 2, 1), (0, -2, 1)]


# --- ground-truth geometry (ONLY for honest clearance verification, NOT used
#     by the robot's perception/planning) -- matches omnitug500_explore.omniworld ----------
GT_RECTS = [
    (-6.2, 6.2, 4.9, 5.1), (-6.2, 6.2, -5.1, -4.9),      # N, S walls
    (5.9, 6.1, -5, 5), (-6.1, -5.9, -5, 5),              # E, W walls
    (-0.1, 0.1, -5, -0.9), (-0.1, 0.1, 0.9, 5),          # divider (doorway gap)
    (2.5, 6.0, 1.4, 1.6),                                # alcove
    (-3.85, -3.15, 1.65, 2.35),                          # box L1
    (-2.75, -2.25, -3.4, -1.6),                          # shelf L2
    (2.85, 3.55, -3.15, -2.45),                          # box R1
    (4.3, 4.9, -1.3, -0.7),                              # box R3
]
GT_CYLS = [(1.3, 3.5, 0.25)]                             # pillar R2


def clearance(px, py):
    """Distance from (px,py) to the nearest real obstacle surface (>=0)."""
    best = 1e9
    for xmin, xmax, ymin, ymax in GT_RECTS:
        dx = max(xmin - px, 0.0, px - xmax)
        dy = max(ymin - py, 0.0, py - ymax)
        best = min(best, math.hypot(dx, dy))
    for cx, cy, r in GT_CYLS:
        best = min(best, max(0.0, math.hypot(px - cx, py - cy) - r))
    return best


def clampf(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def w2c(x, y):
    return int((x - X0) / RES), int((y - Y0) / RES)


def c2w(ix, iy):
    return X0 + (ix + 0.5) * RES, Y0 + (iy + 0.5) * RES


def inb(ix, iy):
    return 0 <= ix < NX and 0 <= iy < NY


class Grid:
    def __init__(self):
        self.lo = [0.0] * (NX * NY)

    def _i(self, ix, iy):
        return iy * NX + ix

    def upd(self, ix, iy, d):
        if inb(ix, iy):
            k = self._i(ix, iy)
            self.lo[k] = clampf(self.lo[k] + d, -L_CLAMP, L_CLAMP)

    def force_free(self, ix, iy):
        if inb(ix, iy):
            self.lo[self._i(ix, iy)] = -L_CLAMP

    def cls(self, ix, iy):
        if not inb(ix, iy):
            return 1
        v = self.lo[self._i(ix, iy)]
        return 0 if v < T_FREE else 1 if v > T_OCC else -1

    def stamp_free(self, x, y, rad):
        rc = int(math.ceil(rad / RES))
        cx, cy = w2c(x, y)
        for dy in range(-rc, rc + 1):
            for dx in range(-rc, rc + 1):
                if dx * dx + dy * dy <= rc * rc:
                    self.force_free(cx + dx, cy + dy)

    def ray(self, x0c, y0c, x1c, y1c, hit):
        cells = []
        dx, dy = abs(x1c - x0c), abs(y1c - y0c)
        sx, sy = (1 if x0c < x1c else -1), (1 if y0c < y1c else -1)
        err = dx - dy
        cx, cy = x0c, y0c
        n = 0
        while True:
            cells.append((cx, cy))
            if (cx == x1c and cy == y1c) or n > 250:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
            n += 1
        for (cx, cy) in cells[:-1]:
            self.upd(cx, cy, -L_FREE)
        ex, ey = cells[-1]
        self.upd(ex, ey, L_OCC if hit else -L_FREE)

    def frontiers(self):
        out = []
        for iy in range(1, NY - 1):
            for ix in range(1, NX - 1):
                if self.cls(ix, iy) != 0:
                    continue
                if (self.cls(ix + 1, iy) == -1 or self.cls(ix - 1, iy) == -1 or
                        self.cls(ix, iy + 1) == -1 or self.cls(ix, iy - 1) == -1):
                    out.append((ix, iy))
        return out

    def inflated_blocked(self):
        rad = int(math.ceil(INFLATE / RES))
        disk = [(dx, dy) for dy in range(-rad, rad + 1) for dx in range(-rad, rad + 1)
                if dx * dx + dy * dy <= rad * rad]
        blocked = set()
        for iy in range(NY):
            for ix in range(NX):
                if self.cls(ix, iy) == 1:
                    for dx, dy in disk:
                        blocked.add((ix + dx, iy + dy))
        return blocked


def cluster(cells):
    """8-connected clustering of a set of frontier cells."""
    s = set(cells)
    seen = set()
    clusters = []
    for c in cells:
        if c in seen:
            continue
        stack = [c]
        seen.add(c)
        comp = []
        while stack:
            cx, cy = stack.pop()
            comp.append((cx, cy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (cx + dx, cy + dy)
                    if n in s and n not in seen:
                        seen.add(n)
                        stack.append(n)
        clusters.append(comp)
    return clusters


def quads_stanza(quads, dcell, z, color, transp, emis):
    """Flat colored quads at height z as one IndexedFaceSet Shape (up-facing)."""
    if not quads:
        return ("Pose { children [ Shape { appearance PBRAppearance { transparency 1 } "
                "geometry IndexedFaceSet { coord Coordinate { point [ 0 0 -9, 0 0 -9, 0 0 -9 ] } "
                "coordIndex [ 0 1 2 -1 ] } } ] }")
    pts, idx = [], []
    for (wx, wy) in quads:
        b = len(pts)
        pts += [(wx, wy), (wx + dcell, wy), (wx + dcell, wy + dcell), (wx, wy + dcell)]
        idx += [b, b + 1, b + 2, -1, b, b + 2, b + 3, -1]
    point_str = ", ".join(f"{x:.3f} {y:.3f} {z:.3f}" for x, y in pts)
    index_str = " ".join(str(v) for v in idx)
    return (
        "Pose { children [ Shape { "
        f"appearance PBRAppearance {{ baseColor {color} transparency {transp} "
        f"metalness 0 roughness 1 emissiveColor {emis} }} "
        f"geometry IndexedFaceSet {{ coord Coordinate {{ point [ {point_str} ] }} "
        f"coordIndex [ {index_str} ] }} }} ] }}"
    )


def astar(start, goal, blocked):
    if start == goal:
        return [start]
    openh = [(0, start)]
    came = {start: None}
    g = {start: 0.0}
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while openh:
        _, cur = heapq.heappop(openh)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            return path[::-1]
        cx, cy = cur
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not inb(nx, ny) or (nx, ny) in blocked:
                continue
            step = 1.4142 if dx and dy else 1.0
            ng = g[cur] + step
            if (nx, ny) not in g or ng < g[(nx, ny)]:
                g[(nx, ny)] = ng
                h = math.hypot(nx - goal[0], ny - goal[1])
                heapq.heappush(openh, (ng + h, (nx, ny)))
                came[(nx, ny)] = cur
    return None


def main():
    # fresh snapshots/log for this run
    try:
        for fn in os.listdir(_OUT):
            if fn.startswith("grid_") or fn.startswith("view_"):
                os.remove(os.path.join(_OUT, fn))
    except Exception:
        pass
    _boot("controller start")
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    dt = ts / 1000.0
    PHYSICS = "--physics" in sys.argv   # dynamic Newton body (setVelocity) vs
    _boot(f"mode={'PHYSICS' if PHYSICS else 'KINEMATIC'}")   # kinematic teleport

    lidars = {}
    for name in SCANNERS:
        dev = robot.getDevice(name)
        if dev is not None:
            dev.enable(ts)
            lidars[name] = dev
    if not lidars:
        _boot("no lidars -- abort")
        return

    me = robot.getSelf()
    my_tf, my_rf = me.getField("translation"), me.getField("rotation")
    tug = robot.getFromDef("OMNITUG500")
    tug_tf = tug.getField("translation") if tug else None
    tug_rf = tug.getField("rotation") if tug else None
    root_children = robot.getRoot().getField("children")
    map_nodes = {"free": None, "occ": None}

    grid = Grid()
    # start from the rover's actual spawn pose (works for any world/size)
    _sp, _so = me.getPosition(), me.getOrientation()
    x, y, yaw_z = _sp[0], _sp[1], math.atan2(_so[3], _so[0])
    traj = [(x, y)]
    telem = []   # per-tick (t, x, y, yaw, v_cmd, w_cmd, actual_speed, actual_w)
    prev_omega = 0.0   # for the steering rate-limit
    reversing = 0      # back-up-recovery countdown (ticks) when wedged
    blacklist = []     # [[wx, wy, expire_step, hits]] wedge spots to avoid; a spot
                       # wedged >=3 times is treated as a permanently unreachable
                       # pocket (excluded even from the last-resort fallback)
    unwedge_turn = 1.0   # +1 / -1: which way to rotate while backing out of a wedge
    target = None
    path = []
    step_count = 0
    dump_idx = 0
    done = False
    last_known = 0
    stall = 0
    last_choose = -999
    no_target = 0
    min_clear = 1e9          # min rover-centre clearance to any real obstacle
    guard_hits = 0           # times the collision guard blocked a move
    stuck_guard = 0          # consecutive guard-blocked ticks
    done_hold = 0            # consecutive idle ticks while exploration is complete
    prog_x, prog_y, prog_step = 0.0, 0.0, 0   # last spot real forward progress was made
    win_step, win_known, win_x, win_y = 0, 0, 0.0, 0.0   # coverage-complete window baseline
    # --- SLAM state ---
    tx = ty = tyaw = 0.0           # latest TRUE (simulator) pose
    est_x = est_y = est_yaw = 0.0  # estimated pose (odometry + scan-matching)
    ptx = pty = ptyaw = 0.0        # previous true pose (for the odometry delta)
    slam_init = False
    odo = random.Random(ODO_SEED)
    lidar_rng = random.Random(LIDAR_SEED)   # range-noise / dropout stream
    sm_field, sm_occ_step = {}, -10**9   # cached likelihood field for the matcher
    drift_max = 0.0                # worst |estimated - true| position error seen

    def collides_disc(px, py):
        """True if any KNOWN occupied cell lies within R_GUARD of (px, py)."""
        rc = int(math.ceil(R_GUARD / RES))
        ci, cj = w2c(px, py)
        for jy in range(cj - rc, cj + rc + 1):
            for ix in range(ci - rc, ci + rc + 1):
                if grid.cls(ix, jy) == 1:
                    wx, wy = c2w(ix, jy)
                    if math.hypot(wx - px, wy - py) <= R_GUARD:
                        return True
        return False

    def sense_and_map():
        cyaw, syaw = math.cos(yaw_z), math.sin(yaw_z)
        near = [1e9, 1e9, 1e9]   # [ahead, front-left, front-right]
        near_hits = []           # LIVE returns within 1.2 m (reactive guard)
        fwd = yaw_z + math.pi / 2.0
        for name, dev in lidars.items():
            ox, oy, phi_m = SCANNERS[name]
            lwx = x + (cyaw * ox - syaw * oy)
            lwy = y + (syaw * ox + cyaw * oy)
            lyaw = yaw_z + phi_m
            clw, slw = math.cos(lyaw), math.sin(lyaw)
            fov = dev.getFov()
            res = dev.getHorizontalResolution()
            maxr, minr = dev.getMaxRange(), dev.getMinRange()
            dth = -fov / res
            th0 = fov / 2.0 + dth / 2.0
            rng = dev.getRangeImage()
            ocx, ocy = w2c(lwx, lwy)
            for i in range(0, res, RAY_STRIDE):
                r = rng[i] if i < len(rng) else maxr
                hit = math.isfinite(r) and (minr <= r <= maxr * 0.999)
                if hit:                              # real-sensor imperfection
                    if LIDAR_DROPOUT and lidar_rng.random() < LIDAR_DROPOUT:
                        hit = False
                    elif LIDAR_NOISE:
                        r += lidar_rng.gauss(0.0, LIDAR_NOISE)
                rr = r if hit else maxr
                th = th0 + i * dth
                px, py = rr * math.cos(th), rr * math.sin(th)
                wx = lwx + (clw * px - slw * py)
                wy = lwy + (slw * px + clw * py)
                grid.ray(ocx, ocy, *w2c(wx, wy), hit=hit)
                if hit:
                    rel = wrap(math.atan2(wy - y, wx - x) - fwd)
                    d = math.hypot(wx - x, wy - y)
                    if d < 1.2:
                        near_hits.append((wx, wy))
                    if abs(rel) < 0.5:
                        near[0] = min(near[0], d)
                    if 0.15 < rel < 1.3:
                        near[1] = min(near[1], d)
                    elif -1.3 < rel < -0.15:
                        near[2] = min(near[2], d)
        grid.stamp_free(x, y, ROVER_FREE_R)
        return near, near_hits

    def read_scan_body():
        """Current lidar hits as (bx, by) points in the ROBOT-BODY frame (no pose
        dependence) -- the point cloud the scan-matcher aligns to the map."""
        pts = []
        for name, dev in lidars.items():
            mx, my, phi = SCANNERS[name]                 # sensor mount in body frame
            cphi, sphi = math.cos(phi), math.sin(phi)
            fov = dev.getFov(); res = dev.getHorizontalResolution()
            maxr, minr = dev.getMaxRange(), dev.getMinRange()
            dth = -fov / res; th0 = fov / 2.0 + dth / 2.0
            rng = dev.getRangeImage()
            for i in range(0, res, SM_STRIDE):
                r = rng[i] if i < len(rng) else maxr
                if not (math.isfinite(r) and minr <= r <= maxr * 0.999):
                    continue
                if LIDAR_DROPOUT and lidar_rng.random() < LIDAR_DROPOUT:
                    continue                          # real-sensor dropout
                if LIDAR_NOISE:
                    r += lidar_rng.gauss(0.0, LIDAR_NOISE)
                th = th0 + i * dth
                sx, sy = r * math.cos(th), r * math.sin(th)        # sensor frame
                pts.append((mx + cphi * sx - sphi * sy, my + sphi * sx + cphi * sy))
        return pts

    def scan_match(px, py, pyaw):
        """Correlative scan-to-map matcher (Hector/Cartographer-style local SLAM):
        align the current scan to a GRADED occupancy likelihood field by a two-stage
        coarse->fine search over (dx, dy, dyaw). The graded field gives the objective
        a gradient so the optimum localises below one cell; the fine stage refines it.
        Offsets are evaluated centre-out with a strict '>' so ties keep the smallest
        correction (no directional bias)."""
        nonlocal sm_field, sm_occ_step
        body = read_scan_body()
        if len(body) < 25:
            return px, py, pyaw
        if step_count - sm_occ_step >= SM_OCC_REFRESH:
            field = {}; lo = grid.lo
            for k in range(NX * NY):
                if lo[k] > T_OCC:
                    cx, cy = k % NX, k // NX
                    for dx, dy, w in SM_KERNEL:
                        c = (cx + dx, cy + dy)
                        if field.get(c, 0) < w:
                            field[c] = w
            sm_field, sm_occ_step = field, step_count
        field = sm_field
        if len(field) < 90:
            return px, py, pyaw
        g = field.get

        def search(cx, cy, cyaw, dstep, astep):
            offs = (0.0, -dstep, dstep, -2 * dstep, 2 * dstep)   # centre-out
            best_s, best = -1, (cx, cy, cyaw)
            for da in (0.0, -astep, astep, -2 * astep, 2 * astep):
                ca, sa = math.cos(cyaw + da), math.sin(cyaw + da)
                rot = [(ca * bx - sa * by, sa * bx + ca * by) for bx, by in body]
                for ddx in offs:
                    ox_ = cx + ddx
                    for ddy in offs:
                        oy_ = cy + ddy
                        s = 0
                        for rx, ry in rot:
                            s += g((int((ox_ + rx - X0) / RES), int((oy_ + ry - Y0) / RES)), 0)
                        if s > best_s:
                            best_s, best = s, (ox_, oy_, wrap(cyaw + da))
            return best_s, best

        _, (cx, cy, cyaw) = search(px, py, pyaw, 0.05, 0.025)        # coarse
        fs, best = search(cx, cy, cyaw, 0.018, 0.009)               # fine
        if fs < SM_MIN_AVG * len(body):     # not enough scan support -> trust odometry
            return px, py, pyaw
        return best

    def plan_to(cell, blocked):
        # Free-only BFS path refresh to a committed target (never through unknown).
        start = w2c(x, y)
        if start in blocked or grid.cls(*start) != 0:
            best = None
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    s = (start[0] + dx, start[1] + dy)
                    if inb(*s) and s not in blocked and grid.cls(*s) == 0:
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, s)
            if best:
                start = best[1]
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        came = {start: None}
        dq = deque([start])
        while dq:
            cur = dq.popleft()
            if cur == cell:
                break
            cx, cy = cur
            for dx, dy in nbrs:
                n = (cx + dx, cy + dy)
                if (inb(*n) and n not in came and n not in blocked
                        and grid.cls(*n) == 0):
                    came[n] = cur
                    dq.append(n)
        if cell not in came:
            return None
        path = []
        cur = cell
        while cur is not None:
            path.append(c2w(*cur))
            cur = came[cur]
        return path[::-1]

    def choose_target():
        # One BFS from the rover over non-blocked cells gives reachability AND a
        # path tree; the target is the nearest REACHABLE frontier (free cell next
        # to unknown) at least MIN_FR_DIST away. Robust to a jagged range arc.
        blocked = grid.inflated_blocked()
        start = w2c(x, y)
        if start in blocked or grid.cls(*start) != 0:
            best = None
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    s = (start[0] + dx, start[1] + dy)
                    if inb(*s) and s not in blocked and grid.cls(*s) == 0:
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, s)
            if best:
                start = best[1]
        came = {start: None}
        dq = deque([start])
        nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while dq:
            cx, cy = dq.popleft()
            for dx, dy in nbrs:
                n = (cx + dx, cy + dy)
                # SAFE exploration: only traverse confirmed-FREE, non-inflated
                # cells -- never plan a path through unknown space (which may
                # hide an obstacle) or through an obstacle's inflation halo.
                if (inb(*n) and n not in came and n not in blocked
                        and grid.cls(*n) == 0):
                    came[n] = (cx, cy)
                    dq.append(n)
        md = MIN_FR_DIST / RES
        # blacklist filtering (this is what breaks the reverse->replan->rewedge loop):
        #  - HARD: a spot wedged >=3 times is a permanently unreachable pocket -> never
        #    target it, even as a last resort, so the rover can declare the map done
        #    instead of looping there forever.
        #  - SOFT: a spot wedged <3 times is avoided until its entry expires, so a
        #    genuinely-reachable frontier gets retried once the map fills in around it.
        def _hard(c):
            wx, wy = c2w(*c)
            return any(h >= 5 and (wx - bx) ** 2 + (wy - by) ** 2 < 0.81
                       for bx, by, exp, h in blacklist)
        def _soft(c):
            wx, wy = c2w(*c)
            return any(exp > step_count and (wx - bx) ** 2 + (wy - by) ** 2 < 0.81
                       for bx, by, exp, h in blacklist)
        all_cands = [c for c in grid.frontiers()
                     if c in came and math.hypot(c[0] - start[0], c[1] - start[1]) >= md
                     and not _hard(c)]
        cands = [c for c in all_cands if not _soft(c)]
        if not cands:
            cands = all_cands     # everything left is only soft-blacklisted -> retry
        if not cands:
            _boot(f"  choose: no reachable frontier (reachable={len(came)} "
                  f"frontiers={len(grid.frontiers())})")
            return None, []
        goal = min(cands, key=lambda c: (c[0] - start[0]) ** 2 + (c[1] - start[1]) ** 2)
        path = []
        cur = goal
        while cur is not None:
            path.append(c2w(*cur))
            cur = came[cur]
        path.reverse()
        return goal, path

    def dump():
        nonlocal dump_idx
        g2 = [[grid.cls(ix, iy) for ix in range(NX)] for iy in range(NY)]
        known = sum(1 for row in g2 for v in row if v != -1)
        snap = {
            "t_ms": step_count * ts, "res": RES, "x0": X0, "y0": Y0,
            "nx": NX, "ny": NY, "grid": g2,
            "rover": [round(x, 3), round(y, 3), round(yaw_z, 4)],
            "rover_true": [round(tx, 3), round(ty, 3), round(tyaw, 4)],
            "traj": [[round(a, 2), round(b, 2)] for a, b in traj[::2]],
            "path": [[round(a, 2), round(b, 2)] for a, b in path],
            "frontier": list(c2w(*target)) if target else None,
            "known_cells": known, "total_cells": NX * NY,
            "frontier_count": len(grid.frontiers()), "done": done,
            "min_clearance": round(min_clear, 3) if min_clear < 1e8 else None,
            "guard_hits": guard_hits,
            "slam": bool(SLAM), "drift_max": round(drift_max, 4),
        }
        with open(os.path.join(_OUT, f"grid_{dump_idx:03d}.json"), "w") as f:
            json.dump(snap, f)
        with open(os.path.join(_OUT, "latest.json"), "w") as f:
            json.dump(snap, f)
        with open(os.path.join(_OUT, "telem.csv"), "w") as f:
            f.write("t_ms,x,y,yaw,v_cmd,w_cmd,actual_speed,actual_w,tx,ty,tyaw\n")
            for r in telem:
                f.write(",".join(str(c) for c in r) + "\n")
        dump_idx += 1

    def export_map():
        """Export the finished occupancy grid as a standard ROS map_server pair
        (map.pgm + map.yaml) -- the real-world deliverable. These two files load
        directly into a robot's navigation stack (nav2 map_server -> AMCL
        localisation -> global planner), so the map the OMNITUG500 just built is
        immediately deployable. Pixel convention (ROS trinary):
            free = 254 (white), occupied = 0 (black), unknown = 205 (gray).
        Image rows run top (max y) -> bottom; `origin` is the world pose of the
        lower-left pixel, so image <-> world is fully georeferenced."""
        FREE, OCC, UNK = 254, 0, 205
        pix = bytearray()
        free_n = occ_n = unk_n = 0
        for iy in range(NY - 1, -1, -1):          # top row = max y (ROS convention)
            for ix in range(NX):
                c = grid.cls(ix, iy)
                if c == 0:
                    pix.append(FREE); free_n += 1
                elif c == 1:
                    pix.append(OCC); occ_n += 1
                else:
                    pix.append(UNK); unk_n += 1
        pgm = os.path.join(_OUT, "map.pgm")
        with open(pgm, "wb") as f:
            f.write(("P5\n# OMNITUG500 autonomous occupancy map\n%d %d\n255\n"
                     % (NX, NY)).encode("ascii"))
            f.write(bytes(pix))
        with open(os.path.join(_OUT, "map.yaml"), "w") as f:
            f.write(
                "image: map.pgm\n"
                "mode: trinary\n"
                f"resolution: {RES:.6f}\n"
                f"origin: [{X0:.6f}, {Y0:.6f}, 0.000000]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.196\n"
                f"# size: {NX} x {NY} cells  ({NX*RES:.2f} x {NY*RES:.2f} m)\n"
                f"# cells: free={free_n} occupied={occ_n} unknown={unk_n}\n"
            )
        # PROBABILISTIC grayscale map (SLAM-Toolbox style): pixel = 255*(1 - P(occ)),
        # P from the raw log-odds -> soft gradient at edges, the look a roboticist
        # expects from a real SLAM map. (The trinary map.pgm above is the deployable
        # nav artifact; this is the presentation render.)
        prob = bytearray()
        lo = grid.lo
        for iy in range(NY - 1, -1, -1):
            base = iy * NX
            for ix in range(NX):
                v = lo[base + ix]
                p = 1.0 / (1.0 + math.exp(-v))     # P(occupied)
                prob.append(int(round(255 * (1.0 - p))))
        with open(os.path.join(_OUT, "map_prob.pgm"), "wb") as f:
            f.write(("P5\n# OMNITUG500 probabilistic occupancy\n%d %d\n255\n" % (NX, NY)).encode("ascii"))
            f.write(bytes(prob))
        # optional human-readable PNGs (only if Pillow is importable; non-fatal)
        extra = ""
        try:
            from PIL import Image
            Image.frombytes("L", (NX, NY), bytes(pix)).save(os.path.join(_OUT, "map.png"))
            Image.frombytes("L", (NX, NY), bytes(prob)).save(os.path.join(_OUT, "map_prob.png"))
            extra = " + map.png + map_prob.png"
        except Exception:
            pass
        _boot(f"EXPORTED ROS map: {pgm} + map.yaml{extra} "
              f"(free={free_n} occ={occ_n} unk={unk_n})")

    def render_map():
        """Paint the explored map onto the floor in 3D: free=green, walls=red."""
        d, dcell = MAP_DS, MAP_DS * RES
        free, occ = [], []
        for by in range(0, NY, d):
            for bx in range(0, NX, d):
                is_occ = False
                n_free = 0
                for yy in range(by, min(by + d, NY)):
                    for xx in range(bx, min(bx + d, NX)):
                        c = grid.cls(xx, yy)
                        if c == 1:
                            is_occ = True
                        elif c == 0:
                            n_free += 1
                if is_occ:
                    occ.append((X0 + bx * RES, Y0 + by * RES))
                elif n_free > 0:
                    free.append((X0 + bx * RES, Y0 + by * RES))
        specs = [("free", free, MAP_FREE_Z, "0.20 0.85 0.45", 0.6, "0.05 0.14 0.07"),
                 ("occ", occ, MAP_OCC_Z, "0.92 0.22 0.22", 0.2, "0.22 0.0 0.0")]
        for key, quads, z, color, transp, emis in specs:
            try:
                if map_nodes[key] is not None:
                    map_nodes[key].remove()
                root_children.importMFNodeFromString(
                    -1, quads_stanza(quads, dcell, z, color, transp, emis))
                map_nodes[key] = root_children.getMFNode(-1)
            except Exception:
                _boot(f"map render {key} failed:\n" + traceback.format_exc())

    _boot(f"grid {NX}x{NY}; exploring")
    while robot.step(ts) != -1:
        step_count += 1
        if step_count > MAX_STEPS:
            break

        # In physics mode the chassis pose is whatever Newton produced (it may
        # have been blocked by a wall); read it back instead of integrating.
        if PHYSICS:
            p = me.getPosition()
            o = me.getOrientation()
            tx, ty, tyaw = p[0], p[1], math.atan2(o[3], o[0])    # TRUE physical pose
            if SLAM:
                if not slam_init:
                    est_x, est_y, est_yaw = tx, ty, tyaw         # anchor map = start
                    ptx, pty, ptyaw = tx, ty, tyaw
                    slam_init = True
                else:
                    # (1) ODOMETRY: noisy true body-frame delta -> dead-reckon est
                    dwx, dwy, dth = tx - ptx, ty - pty, wrap(tyaw - ptyaw)
                    cpt, spt = math.cos(ptyaw), math.sin(ptyaw)
                    bdx = cpt * dwx + spt * dwy                  # forward
                    bdy = -spt * dwx + cpt * dwy                 # left
                    tr = math.hypot(bdx, bdy)
                    bdx += odo.gauss(0.0, ODO_T * tr + 1e-4)
                    bdy += odo.gauss(0.0, ODO_T * tr + 1e-4)
                    dth += odo.gauss(0.0, ODO_R * abs(dth) + ODO_TR * tr + 1e-5)
                    ce, se = math.cos(est_yaw), math.sin(est_yaw)
                    est_x += ce * bdx - se * bdy
                    est_y += se * bdx + ce * bdy
                    est_yaw = wrap(est_yaw + dth)
                    ptx, pty, ptyaw = tx, ty, tyaw
                    # (2) CORRECTION: scan-match the est pose to the map
                    if step_count > SEED_STEPS and step_count % SM_EVERY == 0:
                        est_x, est_y, est_yaw = scan_match(est_x, est_y, est_yaw)
                    drift_max = max(drift_max, math.hypot(est_x - tx, est_y - ty))
                x, y, yaw_z = est_x, est_y, est_yaw              # NAV + MAP use the estimate
            else:
                x, y, yaw_z = tx, ty, tyaw
        else:
            tx, ty, tyaw = x, y, yaw_z                            # kinematic: est == true

        # PHYSICS-COLLISION PROOF: with OMNISIM_RAM=1, drive straight north (+Y)
        # with NO avoidance. If the physics is real the north wall (surface y=4.9)
        # stops the rover; if it were kinematic it would sail straight through.
        if PHYSICS and os.environ.get("OMNISIM_RAM"):
            me.setVelocity([0.0, V_MAX, 0.0, 0.0, 0.0, 0.0])
            if tug_tf:
                tug_tf.setSFVec3f([x, y, 0.0])
                tug_rf.setSFRotation([0.0, 0.0, 1.0, yaw_z])
            if step_count % 15 == 0:
                _boot(f"RAM t={step_count*ts}ms x={x:.2f} y={y:.2f} z={p[2]:.3f}")
            continue

        near, near_hits = sense_and_map()
        near_ahead, near_left, near_right = near
        if VERIFY_CLEAR and step_count > SEED_STEPS:
            min_clear = min(min_clear, clearance(x, y))
        if not PHYSICS:                 # kinematic: teleport the chassis
            my_tf.setSFVec3f([x, y, 0.0])
            my_rf.setSFRotation([0.0, 0.0, 1.0, yaw_z])
        if tug_tf:                      # visual rover sits at the TRUE physical pose
            tug_tf.setSFVec3f([tx, ty, 0.0])
            tug_rf.setSFRotation([0.0, 0.0, 1.0, tyaw])

        if RENDER_MAP and step_count % MAP_EVERY == 0:
            render_map()

        if step_count <= SEED_STEPS:
            if PHYSICS:
                me.setVelocity([0.0] * 6)   # hold still while the map seeds
            continue

        # stall watchdog: if the map has stopped growing, re-choose a target
        known_now = sum(1 for r in range(NY) for c in range(NX) if grid.cls(c, r) != -1)
        if known_now > last_known + 2:
            last_known = known_now
            stall = 0
        else:
            stall += 1

        # coverage-complete watchdog: if BOTH the known map and the rover position
        # have barely changed over a long window, the navigable area is fully mapped
        # and only unreachable (rack-interior / outside-wall) frontiers remain.
        # Finish cleanly instead of thrashing on phantom frontiers until MAX_STEPS.
        # (Net displacement guards against a false trigger during a long traversal:
        #  while travelling the rover covers metres even when mapping nothing new.)
        if step_count - win_step >= 1500:
            grew = known_now - win_known
            moved = math.hypot(x - win_x, y - win_y)
            if step_count > SEED_STEPS + 1500 and grew < 60 and moved < 1.5:
                dump()
                export_map()
                _boot(f"t={step_count*ts}ms DONE - coverage complete "
                      f"(grew {grew} cells, moved {moved:.1f} m over the window)")
                robot.simulationQuit(0)
                break
            win_step, win_known, win_x, win_y = step_count, known_now, x, y

        # --- target management (commit until reached / invalid; never latch) ---
        if target is not None:
            gx, gy = c2w(*target)
            if math.hypot(gx - x, gy - y) < GOAL_TOL or grid.cls(*target) != 0:
                target = None
        if (target is None and step_count - last_choose >= CHOOSE_RETRY) or stall > STALL_TICKS:
            prev_target = target
            target, path = choose_target()
            last_choose = step_count
            stall = 0
            no_target = 0 if target else no_target + CHOOSE_RETRY
            if target != prev_target:           # fresh target -> fresh progress window
                prog_x, prog_y, prog_step = x, y, step_count
        elif target is not None and step_count % REPLAN_EVERY == 0:
            p = plan_to(target, grid.inflated_blocked())
            if p:
                path = p
            else:
                target = None       # unreachable now -> re-choose next tick

        done = target is None and no_target > COMPLETE_AFTER
        if target is None:
            # idle (no reachable frontier this moment); keep scanning + retry
            if PHYSICS:
                me.setVelocity([0.0] * 6)     # hold still while idle
            if done:
                done_hold += 1
                if done_hold >= COMPLETE_HOLD:   # complete + settled -> finish cleanly
                    dump()
                    export_map()                 # drop the deployable ROS map
                    _boot(f"t={step_count*ts}ms DONE - exploration complete, quitting")
                    robot.simulationQuit(0)
                    break
            if step_count % DUMP_EVERY == 0:
                dump()
                _boot(f"t={step_count*ts}ms IDLE known={known_now} "
                      f"frontiers={len(grid.frontiers())} no_target={no_target} "
                      f"done={done} done_hold={done_hold}")
            continue
        done_hold = 0

        # --- recovery: escape toward OPEN space, then re-plan. ----------------
        # Move along the net repulsion from every live lidar return at once (i.e.
        # directly away from all nearby obstacles). Moving away can only INCREASE
        # clearance, so it's safe to do WITHOUT the guard -- and unlike a heading-
        # aligned reverse it reliably frees the rover even when it has drifted into
        # the guard band where forward / reverse / turn are ALL blocked (the
        # deadlock that once froze it pressed against a rack). Brief omni nudge with
        # a gentle reorient; normal diff-drive resumes once reversing ends.
        if reversing > 0:
            reversing -= 1
            rx = sum(x - hx for hx, hy in near_hits)
            ry = sum(y - hy for hx, hy in near_hits)
            nrm = math.hypot(rx, ry)
            if nrm > 1e-6:
                ex, ey = rx / nrm, ry / nrm
            else:
                fwd = yaw_z + math.pi / 2.0          # nothing near -> back along heading
                ex, ey = -math.cos(fwd), -math.sin(fwd)
            sp, wt = 0.40, 0.6 * unwedge_turn
            if PHYSICS:
                me.setVelocity([sp * ex, sp * ey, 0.0, 0.0, 0.0, wt])
            else:
                x += sp * ex * dt
                y += sp * ey * dt
                yaw_z = wrap(yaw_z + wt * dt)
                tx, ty, tyaw = x, y, yaw_z
            if reversing == 0:
                target = None        # re-plan a fresh route after escaping
                prev_omega = 0.0
            traj.append((x, y))
            telem.append((step_count * ts, round(x, 3), round(y, 3), round(yaw_z, 3),
                          round(-sp, 3), round(wt, 3), round(sp, 3), round(abs(wt), 3),
                          round(tx, 3), round(ty, 3), round(tyaw, 3)))
            continue

        # --- pure-pursuit along the path ---
        carrot = path[-1] if path else (x, y)
        for pt in path:
            if math.hypot(pt[0] - x, pt[1] - y) >= LOOKAHEAD:
                carrot = pt
                break
        desired_yaw = math.atan2(carrot[1] - y, carrot[0] - x) - math.pi / 2.0
        err = wrap(desired_yaw - yaw_z)
        omega = clampf(STEER_KP * err, -W_MAX, W_MAX)
        omega = prev_omega + clampf(omega - prev_omega, -DW_MAX, DW_MAX)   # rate-limit
        prev_omega = omega
        v = V_MAX * max(0.0, 1.0 - abs(err) / 1.6)
        if abs(err) > 1.4 or near_ahead < SAFE_DIST:
            v = 0.0
        # collision guard (shared) + wedge detection
        fwd = yaw_z + math.pi / 2.0
        cx, cy = x + v * math.cos(fwd) * dt, y + v * math.sin(fwd) * dt
        rg2 = R_GUARD * R_GUARD
        live_block = any((hx - cx) ** 2 + (hy - cy) ** 2 < rg2 for hx, hy in near_hits)
        blocked = v > 0.0 and (collides_disc(cx, cy) or live_block)
        if blocked:
            guard_hits += 1
            stuck_guard += 1
        else:
            stuck_guard = 0
        # progress watchdog: note where we last advanced a real step
        if math.hypot(x - prog_x, y - prog_y) > 0.4:
            prog_x, prog_y, prog_step = x, y, step_count
        # Recover if WEDGED (many consecutive guard blocks) OR STALLED (pursuing a
        # target but making no forward progress for ~1.5 s -- e.g. guard-bouncing
        # against a rack end without ever hitting 12-in-a-row). Both back up + turn
        # and SOFT-blacklist this frontier so we try a different one, then retry it
        # later once the map has filled in (a spot wedged 5x becomes permanent).
        if stuck_guard > 12 or (step_count - prog_step) > 130:
            reversing = 45          # ~0.29 m escape -- enough to fully clear the guard band
            stuck_guard = 0
            unwedge_turn = 1.0 if near_left >= near_right else -1.0
            if target is not None:
                bx, by = c2w(*target)
                for e in blacklist:
                    if (e[0] - bx) ** 2 + (e[1] - by) ** 2 < 0.81:
                        e[2] = step_count + 900
                        e[3] += 1
                        break
                else:
                    blacklist.append([bx, by, step_count + 900, 1])
            target = None
            prog_x, prog_y, prog_step = x, y, step_count
        if PHYSICS:
            vv = 0.0 if blocked else v
            fwd_cmd = tyaw + math.pi / 2.0        # robot moves along its TRUE heading
            me.setVelocity([vv * math.cos(fwd_cmd), vv * math.sin(fwd_cmd), 0.0, 0.0, 0.0, omega])
        elif not blocked:
            yaw_z = wrap(yaw_z + omega * dt)
            fn = yaw_z + math.pi / 2.0
            x += v * math.cos(fn) * dt
            y += v * math.sin(fn) * dt
            tx, ty, tyaw = x, y, yaw_z
        traj.append((x, y))         # estimated path (overlaid on the estimate-frame map)
        if PHYSICS:
            av = me.getVelocity()
            asp, awz = math.hypot(av[0], av[1]), av[5]
        else:
            asp, awz = v, omega
        telem.append((step_count * ts, round(x, 3), round(y, 3), round(yaw_z, 3),
                      round(v, 3), round(omega, 3), round(asp, 3), round(awz, 3),
                      round(tx, 3), round(ty, 3), round(tyaw, 3)))

        if step_count % DUMP_EVERY == 0:
            dump()
            _boot(f"t={step_count*ts}ms pos=({x:.1f},{y:.1f}) known={known_now} "
                  f"min_clear={min_clear:.2f}m guard_hits={guard_hits} done={done}")

    # loop ended (MAX_STEPS reached or sim closed before 'done') -> still save the map
    dump()
    export_map()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        _boot("FATAL:\n" + traceback.format_exc())
        raise
