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

"""deep_robotics_crawl -- a scripted, statically stable crawl for the Deep
Robotics Lite3 / X30 on real contact physics.

WHAT THIS IS, AND IS NOT. This is a scripted foot-space crawl (creep) gait:
one leg swings at a time while the other three stay planted, every foot
target goes through a closed-form 3-DoF leg IK, and the twelve position
servos track the result. The body is NOT pinned or translated by a
supervisor -- the robot moves because its planted feet push on the ground.
Its terrain awareness is a MAP, not perception: when the world has an
ElevationGrid it is read once through the Supervisor (standing in for the
elevation map a real robot builds from depth sensing) and the gait plants
its feet on it, keeps the body at a set height above the mean ground under
the feet, pitches to sustained grades, picks the flattest of three footholds
along the stride, lifts each swing over the highest ground on its path and
slows on descents (TerrainCrawl). A heading hold steers from the robot's own
pose. There is no policy, no contact sensing and no balance feedback (a
measured-tilt loop exists and is OFF: it rocked the robot over). It is not
learned locomotion and must not be reported as such; the learned quadruped
walks in this tree are the Go2 / OmniQuad Shadowing policies
(docs/developer/rl-current-state.md).

The gait model is the Unitree B2 crawl of
projects/policies/control/gait/b2_crawl_gait.py (stance foot slides back at
exactly -vx, swing foot on a quintic catch-up arc, creep order FL -> RR -> FR
-> RL at duty 0.85, stride ramped in from a standing start), re-parameterised
for the Deep Robotics leg geometry and joint conventions. Deep Robotics URDFs
put the hip-roll axis on (-1,0,0) and the hip-pitch / knee axes on (0,-1,0),
the mirror of Unitree's (1,0,0) / (0,1,0), so every IK angle is negated on
the way to the motor. `python deep_robotics_crawl.py --selftest` checks that
mapping by running the IK output through a forward kinematics built from the
URDF chain (no simulator needed).

Robot selection: controllerArgs ["--robot" "lite3"|"x30"]. Speed and cadence
can be overridden with --vx / --freq (m/s, Hz).

Trace (for validation): OMNISIM_PROBE_OUT=<file> writes one line per step
with the body position, up-vector, gait phase and joint angles for
OMNISIM_PROBE_STEPS steps (default 5000); needs `supervisor TRUE`.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

LEGS = ("FL", "FR", "RL", "RR")


# ── Robot geometry (from the URDFs in projects/robots/deep_robotics) ───────
ROBOTS = {
    "lite3": dict(
        prefixes={"FL": "FL", "FR": "FR", "RL": "HL", "RR": "HR"},
        joints=("HipX_joint", "HipY_joint", "Knee_joint"),
        hip=dict(FL=(0.1745, 0.062, 0.0), FR=(0.1745, -0.062, 0.0),
                 RL=(-0.1745, 0.062, 0.0), RR=(-0.1745, -0.062, 0.0)),
        hipy_offset=0.09735, l1=0.20, l2=0.21012,
        # gait defaults
        body_height=0.235, lateral_y=0.185, vx=0.10, freq=0.8, duty=0.85,
        step_height=0.05, ramp_s=1.5, x0=0.0,
    ),
    "x30": dict(
        prefixes={"FL": "FL", "FR": "FR", "RL": "HL", "RR": "HR"},
        joints=("HipX_joint", "HipY_joint", "Knee_joint"),
        hip=dict(FL=(0.291, 0.08, 0.0), FR=(0.291, -0.08, 0.0),
                 RL=(-0.291, 0.08, 0.0), RR=(-0.291, -0.08, 0.0)),
        hipy_offset=0.11675, l1=0.30, l2=0.31,
        body_height=0.36, lateral_y=0.22, vx=0.14, freq=0.7, duty=0.85,
        step_height=0.07, ramp_s=1.5, x0=0.0,
    ),
}
HIPY_SIGN = {"FL": +1.0, "FR": -1.0, "RL": +1.0, "RR": -1.0}
# Creep order FL -> RR -> FR -> RL, one quarter cycle apart (b2_crawl_gait).
CRAWL_OFFSET = {"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75}
# Diagonal-pair trot (b2_trot_gait's phasing). Not statically stable -- two
# feet planted between the four-foot windows -- and not used by this
# controller; the OmniLink quadruped bridge selects it by name. At duty 0.6
# phase 0.05 still has all four feet planted, so QS_PHASE serves both.
TROT_OFFSET = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
# Gait clock phase (rad) at which all four feet are planted (duty 0.85).
QS_PHASE = 0.05 * 2.0 * math.pi


class Gait:
    """The b2_crawl_gait foot-space model with the geometry as parameters."""

    def __init__(self, cfg: dict, vx=None, freq=None, offsets=None):
        self.cfg = cfg
        self.offsets = dict(offsets if offsets is not None else CRAWL_OFFSET)
        self.vx = float(vx if vx is not None else cfg["vx"])
        self.freq = float(freq if freq is not None else cfg["freq"])
        self.duty = float(cfg["duty"])
        self.step_height = float(cfg["step_height"])
        self.body_height = float(cfg["body_height"])
        self.lateral_y = float(cfg["lateral_y"])
        self.ramp_s = float(cfg["ramp_s"])
        self.x0 = float(cfg["x0"])

    @property
    def step_len(self):
        return self.vx * self.duty / self.freq

    @staticmethod
    def _quintic(s):
        return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)

    def _foot_x_z(self, leg_phi, t_since_start):
        L = self.step_len
        if t_since_start is not None and self.ramp_s > 0:
            L *= min(1.0, max(0.0, t_since_start / self.ramp_s))
        if leg_phi < self.duty:                       # stance: slide back at -vx
            s = leg_phi / self.duty
            return L * (0.5 - s), 0.0, 0.0
        s = (leg_phi - self.duty) / (1.0 - self.duty)  # swing: quintic catch-up
        return L * (self._quintic(s) - 0.5), self.step_height * math.sin(math.pi * s) ** 2, math.sin(math.pi * s)

    def leg_ik(self, leg, tx, ty, tz):
        """Body-frame foot target -> (hip roll, hip pitch, knee) in the
        Unitree convention (roll about +X, pitch about +Y, knee about +Y,
        knee angle <= 0 for the '<' leg). Closed form, from b2_kinematics."""
        cfg = self.cfg
        Hx, Hy, _ = cfg["hip"][leg]
        Tx, Ty, Tz = tx - Hx, ty - Hy, tz
        H = HIPY_SIGN[leg] * cfg["hipy_offset"]
        L1, L2 = cfg["l1"], cfg["l2"]
        r_yz = math.hypot(Ty, Tz)
        a = math.atan2(Tz, Ty) + math.acos(max(-1.0, min(1.0, H / max(r_yz, 1e-9))))
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        sa, ca = math.sin(a), math.cos(a)
        Qx = Tx
        Qz = -Ty * sa + Tz * ca
        Q2 = Qx * Qx + Qz * Qz
        C = (Q2 - L1 * L1 - L2 * L2) / (2.0 * L2)
        g = -math.acos(max(-1.0, min(1.0, C / L1)))
        Sx = -L2 * math.sin(g)
        Sz = -L1 - L2 * math.cos(g)
        b = math.atan2(Sz, Sx) - math.atan2(Qz, Qx)
        return a, b, g

    def _yaw_sweep(self, leg_phi, wz):
        """Tangential stance sweep for a yaw rate wz (rad/s): the planted feet
        rotate about the body z during stance, so the body turns (b2_crawl_gait)."""
        if wz == 0.0:
            return 0.0
        sweep = wz * self.duty / self.freq
        if leg_phi < self.duty:
            return sweep * (0.5 - leg_phi / self.duty)
        s = (leg_phi - self.duty) / (1.0 - self.duty)
        return sweep * (self._quintic(s) - 0.5)

    def stride(self, phase_rad, t_since_start, wz=0.0):
        """Per leg: body-frame foot (fx, fy) on the stride, the leg's cycle phase
        in [0,1), the swing progress s in [0,1] (None in stance) and the
        flat-ground swing lift dz. z is decided by the caller (flat: -body_height
        + dz; terrain-aware: see TerrainCrawl)."""
        phi = (phase_rad / (2.0 * math.pi)) % 1.0
        out = {}
        for leg in LEGS:
            leg_phi = (phi + self.offsets[leg]) % 1.0
            dx, dz, sw = self._foot_x_z(leg_phi, t_since_start)
            fx = self.cfg["hip"][leg][0] + self.x0 + dx
            fy = self.lateral_y * HIPY_SIGN[leg]
            ang = self._yaw_sweep(leg_phi, wz)
            if ang != 0.0:
                ca, sa = math.cos(ang), math.sin(ang)
                fx, fy = fx * ca - fy * sa, fx * sa + fy * ca
            s = None if leg_phi < self.duty else (leg_phi - self.duty) / (1.0 - self.duty)
            out[leg] = (fx, fy, leg_phi, s, dz)
        return out

    def touchdown_xy(self, leg, t_since_start, wz=0.0):
        """Body-frame (fx, fy) where this leg's swing will end."""
        L = self.step_len
        if t_since_start is not None and self.ramp_s > 0:
            L *= min(1.0, max(0.0, t_since_start / self.ramp_s))
        fx = self.cfg["hip"][leg][0] + self.x0 + 0.5 * L
        fy = self.lateral_y * HIPY_SIGN[leg]
        ang = self._yaw_sweep(self.duty, wz)   # sweep at the end of stance / start of swing
        if ang != 0.0:
            ca, sa = math.cos(ang), math.sin(ang)
            fx, fy = fx * ca - fy * sa, fx * sa + fy * ca
        return fx, fy

    def foot_targets(self, phase_rad, t_since_start, wz=0.0):
        """Flat-ground body-frame foot targets (4 x (x, y, z)) + swing weights."""
        st = self.stride(phase_rad, t_since_start, wz)
        feet = {leg: (fx, fy, -self.body_height + dz) for leg, (fx, fy, _p, _s, dz) in st.items()}
        swings = {leg: (math.sin(math.pi * s) if s is not None else 0.0) for leg, (_x, _y, _p, s, _dz) in st.items()}
        return feet, swings

    def joints_for(self, feet):
        """Body-frame feet -> per leg (hip_x, hip_y, knee) in the DEEP ROBOTICS
        joint convention: every axis is the mirror of Unitree's, so every
        angle is negated."""
        out = {}
        for leg in LEGS:
            a, b, g = self.leg_ik(leg, *feet[leg])
            out[leg] = (-a, -b, -g)
        return out

    def joint_targets(self, phase_rad, t_since_start, wz=0.0):
        feet, swings = self.foot_targets(phase_rad, t_since_start, wz)
        return self.joints_for(feet), swings

    def standing_pose(self):
        q, _ = self.joint_targets(QS_PHASE, 0.0)
        return q


# ── Terrain awareness ──────────────────────────────────────────────────────
class TerrainMap:
    """The world's ElevationGrid, read once through the Supervisor, sampled
    bilinearly in world coordinates. Stands in for the elevation map a real
    robot builds from its own depth sensing; outside the grid the ground is
    the flat floor at z = 0. `None` when the world has no such grid."""

    @staticmethod
    def load(robot, geom_def, solid_def):
        try:
            g = robot.getFromDef(geom_def)
            s = robot.getFromDef(solid_def)
        except Exception:
            return None
        if g is None:
            return None
        try:
            nx = int(g.getField("xDimension").getSFInt32())
            ny = int(g.getField("yDimension").getSFInt32())
            xs = float(g.getField("xSpacing").getSFFloat())
            ys = float(g.getField("ySpacing").getSFFloat())
            hf = g.getField("height")
            n = int(hf.getCount())
            if n < nx * ny or nx < 2 or ny < 2:
                return None
            h = [float(hf.getMFFloat(i)) for i in range(nx * ny)]
            origin = s.getField("translation").getSFVec3f() if s is not None else (0.0, 0.0, 0.0)
        except Exception as exc:  # noqa: BLE001
            print("[deep_robotics_crawl] terrain map unreadable: %r" % (exc,), file=sys.stderr)
            return None
        return TerrainMap(nx, ny, xs, ys, h, origin)

    def __init__(self, nx, ny, xs, ys, h, origin):
        self.nx, self.ny, self.xs, self.ys, self.h = nx, ny, xs, ys, h
        self.ox, self.oy, self.oz = float(origin[0]), float(origin[1]), float(origin[2])

    def height(self, xw, yw):
        u = (xw - self.ox) / self.xs
        v = (yw - self.oy) / self.ys
        if u <= 0.0 or v <= 0.0 or u >= self.nx - 1 or v >= self.ny - 1:
            return 0.0
        i, j = int(u), int(v)
        fu, fv = u - i, v - j
        H = self.h
        nx = self.nx
        h00 = H[j * nx + i]
        h10 = H[j * nx + i + 1]
        h01 = H[(j + 1) * nx + i]
        h11 = H[(j + 1) * nx + i + 1]
        return self.oz + (h00 * (1 - fu) + h10 * fu) * (1 - fv) + (h01 * (1 - fu) + h11 * fu) * fv

    def max_along(self, x0, y0, x1, y1, n=7):
        return max(self.height(x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n))

    def roughness(self, x, y, r=0.06):
        """How uneven the ground is within a foot's radius of (x, y): the largest
        height difference to the four points r away. 0 on a flat plateau, large
        on a step edge or a facet."""
        h0 = self.height(x, y)
        return max(abs(self.height(x + dx, y + dy) - h0) for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)))


class TerrainCrawl:
    """Turns the flat stride into terrain-following foot targets.

    Planted feet stay where they are in the world, so their body-frame z is
    the terrain height under them minus the terrain height under the body:
    as the body walks up a slope its reference rises and the stance feet
    push it up with it, and the body stays level at body_height above the
    ground beneath it. A swinging foot leaves at its old ground height,
    lands at the ground height of its planned touchdown, and lifts
    step_height above the HIGHEST ground along the path between them, so a
    step or a rock on the way is cleared rather than kicked. Everything the
    swing needs is fixed at lift-off (the touchdown point moves only by the
    body's ~3 cm of travel during the 0.2 s swing). Nothing here measures
    contact; the map is trusted, which is exactly its limit.
    """

    FOOTHOLD_SEARCH = 0.05      # m: candidates at 0 / +-5 cm along the stride around the planned touchdown
    ATTITUDE_GAIN = 0.0         # measured-tilt feedback through the legs. OFF: with 0.3-0.6 the
    #                             stiff position-controlled crawl rocked itself over in the rubble;
    #                             the map alone crossed it (--attitude-gain to experiment)
    DESCENT_SLOWDOWN = 2.5      # speed factor 1 - k*grade on descents (a 12-degree grade halves it)
    ATTITUDE_CLAMP = 0.04       # m: largest per-foot attitude correction
    TILT_LP = 0.05              # per-tick low-pass on the measured tilt (~0.16 s at 125 Hz)
    SLOPE_LOOKAHEAD = 0.8       # m: half-span the grade is read over (a 0.4 m rubble plateau averages out)
    SLOPE_PITCH_FRACTION = 0.7  # how much of the local grade the body pitches to
    SLOPE_MIN_GRADE = 0.06      # grades below this (rubble, ripple) are walked with a level body
    SLOPE_LP = 0.01             # per-tick low-pass on the target pitch (~0.8 s)

    def __init__(self, gait, terrain):
        self.gait = gait
        self.terrain = terrain
        self.plan = {}          # leg -> (h_old, h_next, h_max) for the current swing
        self.offset = {leg: (0.0, 0.0) for leg in LEGS}        # chosen foothold shift, body frame
        self.offset_prev = {leg: (0.0, 0.0) for leg in LEGS}   # the one the foot lifted off from
        self.was_swinging = {leg: False for leg in LEGS}
        self.tilt = [0.0, 0.0]  # low-passed body up-vector x, y components
        self.pitch_des = 0.0    # low-passed target pitch (rad, +nose-down) from the map's slope

    def _pick_foothold(self, leg, tx, ty, bx, by, cy, sy):
        """Foothold selection: of the planned touchdown and its 5 cm
        neighbours (body frame), the one on the flattest ground within a foot
        radius, ties to the nominal. Returns the body-frame shift."""
        best, best_score = (0.0, 0.0), None
        d = self.FOOTHOLD_SEARCH
        # Along the stride only: a sideways shift narrows or widens the support
        # polygon, and the crawl's static margin is small (an 8 cm search in
        # both axes tipped the X30 in the rubble that a 5 cm one had crossed).
        for dx in (0.0, d, -d):
            dy = 0.0
            xw, yw = bx + cy * (tx + dx) - sy * (ty + dy), by + sy * (tx + dx) + cy * (ty + dy)
            score = self.terrain.roughness(xw, yw) + 0.15 * abs(dx)  # prefer nominal
            if best_score is None or score < best_score - 1e-9:
                best, best_score = (dx, dy), score
        return best

    def feet(self, phase, tg, wz, bx, by, yaw, up=None):
        g = self.gait
        st = g.stride(phase, tg, wz)
        cy, sy = math.cos(yaw), math.sin(yaw)
        if up is not None:      # low-pass the measured tilt (an IMU would give this)
            self.tilt[0] += self.TILT_LP * (up[0] - self.tilt[0])
            self.tilt[1] += self.TILT_LP * (up[1] - self.tilt[1])
        # Pass 1: each foot's ground height (planted: the map under it; swinging:
        # the quintic blend from where it left to where it will land) and lift.
        plan = {}
        for leg, (fx, fy, _phi, s, dz) in st.items():
            if s is None:                                      # stance: hold the ground height
                self.was_swinging[leg] = False
                ox, oy = self.offset[leg]
                fx, fy = fx + ox, fy + oy
                xw, yw = bx + cy * fx - sy * fy, by + sy * fx + cy * fy
                plan[leg] = (fx, fy, self.terrain.height(xw, yw), 0.0)
            else:
                if not self.was_swinging[leg]:                 # lift-off: plan the whole swing
                    self.was_swinging[leg] = True
                    self.offset_prev[leg] = self.offset[leg]
                    ox, oy = self.offset_prev[leg]
                    xw0, yw0 = bx + cy * (fx + ox) - sy * (fy + oy), by + sy * (fx + ox) + cy * (fy + oy)
                    h_old = self.terrain.height(xw0, yw0)
                    tx, ty = g.touchdown_xy(leg, tg, wz)
                    self.offset[leg] = self._pick_foothold(leg, tx, ty, bx, by, cy, sy)
                    nx, ny = self.offset[leg]
                    txw, tyw = bx + cy * (tx + nx) - sy * (ty + ny), by + sy * (tx + nx) + cy * (ty + ny)
                    h_next = self.terrain.height(txw, tyw)
                    h_max = max(h_old, h_next, self.terrain.max_along(xw0, yw0, txw, tyw))
                    self.plan[leg] = (h_old, h_next, h_max)
                h_old, h_next, h_max = self.plan.get(leg, (0.0, 0.0, 0.0))
                q = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
                (ox0, oy0), (ox1, oy1) = self.offset_prev[leg], self.offset[leg]
                fx, fy = fx + ox0 + (ox1 - ox0) * q, fy + oy0 + (oy1 - oy0) * q
                base = h_old + (h_next - h_old) * q
                lift = (h_max - base + g.step_height) * math.sin(math.pi * s) ** 2
                plan[leg] = (fx, fy, base, lift)
        # The body rides body_height above the MEAN ground height under its
        # four feet -- continuous, as a robot estimating its height from leg
        # kinematics would see it. (Referencing the map under the body centre
        # jumped a whole step every time the centre crossed a plateau edge and
        # yanked all four stance feet at once; that is what flipped the X30 in
        # the rubble.)
        h_ref = sum(p[2] for p in plan.values()) / 4.0
        self.h_ref = h_ref
        # Slope-parallel body. On a ramp a level body puts the mass over short
        # downhill legs and long uphill ones and the descent tips it (measured:
        # the X30 climbed the 12-degree ramp and flipped going down). Read the
        # slope along the heading from the map and pitch the body to most of it
        # (nose-down when the ground ahead is lower), so the legs stay near
        # equal length and the mass stays centred over the feet.
        L = self.SLOPE_LOOKAHEAD
        h_f = self.terrain.height(bx + cy * L, by + sy * L)
        h_b = self.terrain.height(bx - cy * L, by - sy * L)
        grade = (h_f - h_b) / (2.0 * L)
        if abs(grade) < self.SLOPE_MIN_GRADE:
            grade = 0.0
        pitch_target = -math.atan(grade) * self.SLOPE_PITCH_FRACTION
        self.pitch_des += self.SLOPE_LP * (pitch_target - self.pitch_des)
        # Descents are taken slowly: the gait clock's speed factor for the caller.
        self.speed_factor = max(0.35, 1.0 - self.DESCENT_SLOWDOWN * max(0.0, -grade))
        feet = {}
        for leg, (fx, fy, base, lift) in plan.items():
            z = -g.body_height + (base - h_ref) + lift + self.pitch_des * fx
            # Attitude: the measured tilt (up vector leaning +x = nose-down)
            # minus the intended pitch is taken out by lengthening the legs on
            # the low side and shortening the high side. The map already
            # shapes the body when it is right; this handles slip and
            # compliance.
            if up is not None:
                # World up in body coordinates leans BACKWARD (-x) when the nose
                # is down and toward -y when the left side is down, so the
                # measured nose-down pitch is -tilt_x and left-side-down roll
                # is -tilt_y. (With the sign the other way round this loop is
                # positive feedback: the X30 flipped in the first hills.)
                pitch_err = -self.tilt[0] - math.sin(self.pitch_des)
                roll_meas = -self.tilt[1]
                corr = self.ATTITUDE_GAIN * (pitch_err * fx + roll_meas * fy)
                z -= max(-self.ATTITUDE_CLAMP, min(self.ATTITUDE_CLAMP, corr))
            feet[leg] = (fx, fy, z)
        self.last_feet = feet
        return feet


# ── Forward kinematics on the URDF chain (self-test only) ──────────────────
def _rot(axis, q):
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[c + x * x * t, x * y * t - z * s, x * z * t + y * s],
            [y * x * t + z * s, c + y * y * t, y * z * t - x * s],
            [z * x * t - y * s, z * y * t + x * s, c + z * z * t]]


def _mv(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def foot_fk(cfg, leg, q_hipx, q_hipy, q_knee):
    """Foot position in the body frame through the URDF chain: hip joint at
    cfg.hip (axis -X), thigh joint at (0, +-hipy_offset, 0) (axis -Y), knee at
    (0, 0, -l1) (axis -Y), foot at (0, 0, -l2)."""
    hip = list(cfg["hip"][leg])
    R1 = _rot((-1.0, 0.0, 0.0), q_hipx)
    p = _add(hip, _mv(R1, [0.0, HIPY_SIGN[leg] * cfg["hipy_offset"], 0.0]))
    R2 = [[sum(R1[i][k] * _rot((0.0, -1.0, 0.0), q_hipy)[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    p = _add(p, _mv(R2, [0.0, 0.0, -cfg["l1"]]))
    R3 = [[sum(R2[i][k] * _rot((0.0, -1.0, 0.0), q_knee)[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return _add(p, _mv(R3, [0.0, 0.0, -cfg["l2"]]))


def selftest():
    worst = 0.0
    for name, cfg in ROBOTS.items():
        g = Gait(cfg)
        for phase in (0.0, 0.7, 1.9, 3.3, 4.8, 6.0):
            feet, _ = g.foot_targets(phase, 10.0)
            q, _ = g.joint_targets(phase, 10.0)
            for leg in LEGS:
                fk = foot_fk(cfg, leg, *q[leg])
                err = math.dist(fk, feet[leg])
                worst = max(worst, err)
        sp = g.standing_pose()
        print("%s standing pose FL (hip_x, hip_y, knee) = (%.3f, %.3f, %.3f); step %.3f m at %.2f Hz"
              % (name, *sp["FL"], g.step_len, g.freq))
    print("IK -> URDF-chain FK worst foot error: %.2e m" % worst)
    return worst < 1e-9


# ── Controller ─────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="lite3", choices=sorted(ROBOTS))
    p.add_argument("--vx", type=float, default=None)
    p.add_argument("--freq", type=float, default=None)
    p.add_argument("--settle", type=float, default=1.5, help="seconds to ramp into the crawl stance")
    p.add_argument("--no-steer", action="store_true",
                   help="disable the heading hold (the gait then drifts in yaw, as any open-loop gait does)")
    p.add_argument("--k-heading", type=float, default=1.2, help="yaw-rate gain, rad/s per rad of heading error")
    p.add_argument("--wz-max", type=float, default=0.3, help="yaw-rate clamp, rad/s")
    p.add_argument("--lookahead", type=float, default=1.5, help="metres ahead the centreline is aimed at")
    p.add_argument("--stop-x", type=float, default=None,
                   help="world x at which the crawl blends back to the standing pose and holds (needs supervisor TRUE)")
    p.add_argument("--terrain-def", default="TERRAIN_GEOM", help="DEF of the world's ElevationGrid ('' = walk flat)")
    p.add_argument("--terrain-solid-def", default="TERRAIN", help="DEF of the Solid that places the grid")
    p.add_argument("--step-height", type=float, default=None, help="swing clearance above the highest ground on the path (m)")
    p.add_argument("--body-height", type=float, default=None, help="body height above the ground beneath it (m)")
    p.add_argument("--attitude-gain", type=float, default=None,
                   help="measured-tilt feedback through the legs (default off; 0.3 rocked the X30 over in rubble)")
    p.add_argument("--slope-pitch", type=float, default=None, help="fraction of the local grade the body pitches to (0 = level body)")
    p.add_argument("--slope-lookahead", type=float, default=None, help="half-span (m) the grade is read over")
    p.add_argument("--descent-slowdown", type=float, default=None, help="speed factor 1 - k*grade on descents (0 = none)")
    p.add_argument("--selftest", action="store_true")
    args, _ = p.parse_known_args()
    if args.selftest:
        return 0 if selftest() else 1

    from omnisim import Supervisor
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())
    dt = ts / 1000.0
    cfg = dict(ROBOTS[args.robot])
    if args.step_height is not None:
        cfg["step_height"] = args.step_height
    if args.body_height is not None:
        cfg["body_height"] = args.body_height
    gait = Gait(cfg, vx=args.vx, freq=args.freq)

    motors, sensors = {}, {}
    for leg in LEGS:
        pre = cfg["prefixes"][leg]
        for jname, key in zip(cfg["joints"], ("hip_x", "hip_y", "knee")):
            name = f"{pre}_{jname}_motor"
            m = robot.getDevice(name) or robot.getDevice(name[:-len("_motor")])
            if m is None:
                print(f"[deep_robotics_crawl] missing motor {name}", file=sys.stderr)
                return 1
            s = m.getPositionSensor()
            if s is not None:
                s.enable(ts)
            motors[(leg, key)] = m
            sensors[(leg, key)] = s

    trace_path = os.environ.get("OMNISIM_PROBE_OUT")
    trace_steps = int(os.environ.get("OMNISIM_PROBE_STEPS", "5000"))
    trace = open(trace_path, "w", encoding="utf-8") if trace_path else None
    # Heading hold. The robot's own pose stands in for what an IMU + leg
    # odometry would give a real one: yaw and lateral offset from the +x
    # centreline it started on. It only changes where the feet go (the gait's
    # yaw sweep); nothing writes the body pose.
    me = None
    try:
        me = robot.getSelf()
    except Exception:
        me = None
    steer = (me is not None) and not args.no_steer
    if not steer and not args.no_steer:
        print("[deep_robotics_crawl] no supervisor access: heading hold OFF (set supervisor TRUE)", file=sys.stderr)
    terrain = None
    if me is not None and args.terrain_def:
        terrain = TerrainMap.load(robot, args.terrain_def, args.terrain_solid_def)
    crawl = TerrainCrawl(gait, terrain) if terrain is not None else None
    if crawl is not None:
        if args.attitude_gain is not None:
            crawl.ATTITUDE_GAIN = args.attitude_gain
        if args.slope_pitch is not None:
            crawl.SLOPE_PITCH_FRACTION = args.slope_pitch
        if args.slope_lookahead is not None:
            crawl.SLOPE_LOOKAHEAD = args.slope_lookahead
        if args.descent_slowdown is not None:
            crawl.DESCENT_SLOWDOWN = args.descent_slowdown
    print("[deep_robotics_crawl] terrain map: %s" % (
        "%dx%d grid, %.2f m cells, max height %.3f m -- feet follow it" % (
            terrain.nx, terrain.ny, terrain.xs, max(terrain.h)) if terrain else "none -- flat-ground stride"),
        file=sys.stderr)

    # First tick: read where the engine spawned the joints; ramp from there
    # into the crawl's standing pose, then start the gait clock.
    if robot.step(ts) == -1:
        return 0
    start = {}
    stand = gait.standing_pose()
    for leg in LEGS:
        for i, key in enumerate(("hip_x", "hip_y", "knee")):
            s = sensors[(leg, key)]
            v = s.getValue() if s is not None else float("nan")
            start[(leg, key)] = v if math.isfinite(v) else stand[leg][i]
    t0 = robot.getTime()
    print("[deep_robotics_crawl] %s: crawl at %.2f m/s, %.2f Hz, step %.3f m, body %.3f m; settling %.1f s"
          % (args.robot, gait.vx, gait.freq, gait.step_len, gait.body_height, args.settle), file=sys.stderr)

    k = 0
    x_start = None
    y_line = 0.0
    last_report = 0.0
    wz = 0.0
    stop_t = None          # sim time the stop line was crossed
    speed = 1.0            # 1 = full gait, 0 = standing pose (blend after --stop-x)
    gait_time = 0.0        # the gait clock: runs slower than sim time on descents
    while True:
        t = robot.getTime() - t0
        pos = o = None
        if me is not None:
            pos = me.getPosition()
            o = me.getOrientation()
            if x_start is None:
                x_start, y_line = pos[0], pos[1]
            if args.stop_x is not None and stop_t is None and pos[0] >= args.stop_x:
                stop_t = robot.getTime()
                print("[deep_robotics_crawl] reached x=%.2f m: stopping and holding the stance" % pos[0], file=sys.stderr)
            if stop_t is not None:
                speed = max(0.0, 1.0 - (robot.getTime() - stop_t) / 1.5)
            if steer and speed > 0.0:
                yaw = math.atan2(o[3], o[0])                      # body +x in the world, row-major R
                psi_d = -math.atan2(pos[1] - y_line, args.lookahead)  # aim at the centreline ahead
                err = (psi_d - yaw + math.pi) % (2.0 * math.pi) - math.pi
                wz = max(-args.wz_max, min(args.wz_max, args.k_heading * err))
        if t < args.settle:
            a = t / args.settle
            a = a * a * (3.0 - 2.0 * a)
            for leg in LEGS:
                for i, key in enumerate(("hip_x", "hip_y", "knee")):
                    st = start[(leg, key)]
                    motors[(leg, key)].setPosition(st + (stand[leg][i] - st) * a)
            phase = QS_PHASE
        else:
            # The gait clock advances at the crawl's speed factor (descents are
            # taken slowly), so both the stride and the cadence scale together.
            gait_dt = dt * (getattr(crawl, "speed_factor", 1.0) if crawl is not None else 1.0)
            tg = t - args.settle
            gait_time = min(tg, gait_time + gait_dt) if tg > 0 else 0.0
            phase = QS_PHASE + 2.0 * math.pi * gait.freq * gait_time
            if crawl is not None and pos is not None:
                yaw = math.atan2(o[3], o[0])
                # World up expressed in the BODY frame: R^T (0,0,1) = the third row
                # of the row-major orientation matrix. x > 0 when the nose is down.
                up_body = (o[6], o[7])
                q = gait.joints_for(crawl.feet(phase, gait_time, wz * speed, pos[0], pos[1], yaw, up=up_body))
            else:
                q, _ = gait.joint_targets(phase, gait_time, wz * speed)
            for leg in LEGS:
                for i, key in enumerate(("hip_x", "hip_y", "knee")):
                    # b2_crawl_gait.speed_scale: blend toward the standing pose as
                    # speed -> 0 (after --stop-x), which is statically stable.
                    motors[(leg, key)].setPosition(stand[leg][i] + speed * (q[leg][i] - stand[leg][i]))
        if pos is not None:
            if trace and k < trace_steps:
                qs = " ".join("%.3f" % (sensors[(leg, key)].getValue() if sensors[(leg, key)] else float("nan"))
                              for leg in LEGS for key in ("hip_x", "hip_y", "knee"))
                extra = ""
                if crawl is not None and getattr(crawl, "last_feet", None):
                    extra = " href=%.3f feetz=[%s]" % (getattr(crawl, "h_ref", 0.0),
                                                       " ".join("%.3f" % crawl.last_feet[leg][2] for leg in LEGS))
                trace.write("t=%.3f pos=(%.3f,%.3f,%.3f) up=(%.2f,%.2f,%.2f) phase=%.3f wz=%.3f q=[%s]%s\n"
                            % (robot.getTime(), pos[0], pos[1], pos[2], o[2], o[5], o[8], phase, wz, qs, extra))
                trace.flush()
            if robot.getTime() - last_report >= 5.0:
                last_report = robot.getTime()
                print("[deep_robotics_crawl] t=%.1f s  x=%+.2f m (%.2f m travelled)  y=%+.2f  z=%.3f  up_z=%.2f  wz=%+.2f"
                      % (robot.getTime(), pos[0], pos[0] - x_start, pos[1], pos[2], o[8], wz), file=sys.stderr)
        k += 1
        if robot.step(ts) == -1:
            break
    if trace:
        trace.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
