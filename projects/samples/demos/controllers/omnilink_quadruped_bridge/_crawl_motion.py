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

"""_crawl_motion -- measured walk / turn / walk-to on the Deep Robotics gaits.

The quadruped bridge's `--locomotion crawl|trot` mode. The gait model is the
scripted one of projects/robots/deep_robotics/controllers/deep_robotics_crawl
(closed-form leg IK, twelve position servos): its statically stable creep, or
the same model re-phased as a diagonal trot (GAITS below, both measured).
Nothing here writes the body pose: the robot moves because its planted feet
push on the ground, and a command the ground does not deliver is reported as
not delivered.

What this module adds over that controller is the part an operator talks to:

    CrawlDriver.stand()                  hold the gait's standing pose
    CrawlDriver.walk(distance)           along the current heading, held on
                                         the line it started on
    CrawlDriver.turn(angle)              in place, by the gait's yaw sweep,
                                         with a pivot hold against creep
    CrawlDriver.walk_to(x, y, yaw, via)  turn to face, walk with steering
                                         through the via points without
                                         stopping, optionally turn to a
                                         final heading
    CrawlDriver.halt()                   fade out wherever it is

Every motion ends the same way: the gait blends back into the standing pose,
the body is given a moment to settle, a motion that settled outside its
tolerance gets up to MAX_CORRECTIONS passes toward the SAME target, and only
then is the result measured from the robot's pose into `completed`:
{commanded, achieved, error, settled, corrections, ...}, never an echo of the
request. An order replaced before it finished reports `superseded`.

The pose (x, y, yaw) comes from the Supervisor. On a real robot it would come
from leg odometry and an IMU; it is not perception, and nothing here senses
obstacles.

The state machine is pure Python and runs on the SIM thread only: `tick()`
takes the pose and returns joint targets. `python _crawl_motion.py
--selftest` exercises it against an idealised body with no simulator.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
_CRAWL_DIR = os.path.normpath(os.path.join(
    _HERE, "..", "..", "..", "..", "robots", "deep_robotics", "controllers",
    "deep_robotics_crawl"))
if _CRAWL_DIR not in sys.path:
    sys.path.insert(0, _CRAWL_DIR)

import deep_robotics_crawl as _dr  # noqa: E402  (the gait model + terrain map)

LEGS = _dr.LEGS                       # FL, FR, RL, RR (the crawl's own names)
JOINTS = ("hip_x", "hip_y", "knee")


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# Gait presets, measured on the X30 on flat ground (basicTimeStep 8,
# newtonSubsteps 8, compound colliders, mu 2), 2026-09-26. `vx` / `freq` are
# the STRIDE the legs are asked for; the body delivers less, because the
# stance legs lag their position targets:
#   crawl  0.45 m/s stride at 1.3 Hz -> ~0.22 m/s body, 90 deg in ~13 s
#   trot   1.10 m/s stride at 2.2 Hz -> 5 m in 14.6 s, 90 deg in ~6.5 s
# Neither tipped (body up-vector z >= 0.997 throughout). The crawl keeps three
# feet planted and is the one proven on rough ground (x30_extreme_terrain);
# the trot is faster and is flat-ground only until it is measured elsewhere.
# `lead_*` is how early the stop is started, as seconds at the measured rate:
# a crawl coasts after the gait fades, a trot stops almost where it is.
GAITS = {
    "crawl": dict(offsets=_dr.CRAWL_OFFSET, duty=None, vx=0.45, freq=1.3,
                  wz=0.30, lead_walk_s=0.40, lead_turn_s=0.25),
    "trot":  dict(offsets=_dr.TROT_OFFSET, duty=0.6, vx=1.10, freq=2.2,
                  wz=0.80, lead_walk_s=0.15, lead_turn_s=0.05),
}


class CrawlDriver:
    """Speed-scheduled crawl / trot + the motion verbs. SIM THREAD ONLY."""

    # Blend time from the gait into the standing pose (the terrain controller
    # uses the same 1.5 s after --stop-x) and the rest allowed after it
    # before a motion is measured.
    STOP_BLEND_S = 1.5
    BLEND_UP_S = 0.4
    SETTLE_S = 0.6
    # Low-pass time constants for the commanded body rates: the stance feet
    # slide at exactly -vx, so a step change in vx would jerk every planted
    # foot at once.
    VX_TAU_S = 0.6
    WZ_TAU_S = 0.4
    # Heading hold on a straight walk: aim at the start line this far ahead.
    LOOKAHEAD_M = 1.2
    K_HEADING = 1.2
    # Tolerances a motion is judged against.
    WALK_TOL_M = 0.08
    TURN_TOL_RAD = math.radians(4.0)
    GOTO_TOL_M = 0.15
    GOTO_FACE_RAD = math.radians(20.0)   # turn in place first above this
    VIA_PASS_M = 0.35                    # switch to the next via point this close
    # A motion that settles outside its tolerance gets this many closed-loop
    # correction passes toward the SAME target before it is reported.
    MAX_CORRECTIONS = 2

    def __init__(self, robot_key: str, dt: float, terrain=None,
                 gait: str = "crawl",
                 vx_max: Optional[float] = None, freq: Optional[float] = None,
                 wz_max: Optional[float] = None,
                 lead_walk_s: Optional[float] = None,
                 lead_turn_s: Optional[float] = None):
        preset = GAITS[gait]
        cfg = dict(_dr.ROBOTS[robot_key])
        if preset["duty"] is not None:
            cfg["duty"] = preset["duty"]
        self.robot_key = robot_key
        self.gait_name = gait
        self.dt = float(dt)
        self.vx_max = float(vx_max if vx_max is not None else preset["vx"])
        # The gait's stride length is vx * duty / freq. It is built at the
        # nominal speed; the commanded speed moves `gait.vx` below it.
        self.gait = _dr.Gait(cfg, vx=self.vx_max,
                             freq=freq if freq is not None else preset["freq"],
                             offsets=preset["offsets"])
        self.wz_max = float(wz_max if wz_max is not None else preset["wz"])
        if lead_walk_s is None:
            lead_walk_s = preset["lead_walk_s"]
        if lead_turn_s is None:
            lead_turn_s = preset["lead_turn_s"]
        self.crawl = (_dr.TerrainCrawl(self.gait, terrain)
                      if terrain is not None else None)
        self.stand_q = self.gait.standing_pose()
        # How far ahead of the target to start the stop blend, as a time at
        # the MEASURED rate: the body keeps moving while the gait fades out.
        # (The commanded rate is the wrong ruler -- the stance legs lag their
        # targets and the body delivers about half of it; measured on the
        # X30, 0.224 m/s for a 0.45 m/s stride.)
        self.lead_walk_s = float(lead_walk_s)
        self.lead_turn_s = float(lead_turn_s)
        self._hist: List[Tuple[float, float, float, float, float]] = []   # (t, x, y, yaw_unwrapped, yaw)
        self._yaw_unwrapped = None
        self.v_meas = 0.0         # body speed along its heading, m/s
        self.w_meas = 0.0         # yaw rate, rad/s

        self.blend = 0.0          # 0 = standing pose, 1 = full gait
        self.gait_time = 0.0
        self.phase = _dr.QS_PHASE
        self.vx = 0.0             # low-passed commanded rates
        self.vy = 0.0
        self.wz = 0.0

        self.motion: Optional[dict] = None     # the verb in flight
        self.completed: Optional[dict] = None  # its measured result
        self.seq = 0
        self.sim_time = 0.0

    # ── verbs (call from the sim thread, or under the owner's lock) ───

    def stand(self) -> int:
        return self._begin({"verb": "stand"})

    def walk(self, distance: float) -> int:
        return self._begin({"verb": "walk", "commanded": float(distance),
                            "unit": "m"})

    def turn(self, angle_rad: float) -> int:
        return self._begin({"verb": "turn", "commanded": float(angle_rad),
                            "unit": "rad"})

    def walk_to(self, x: float, y: float, yaw: Optional[float] = None,
                label: Optional[str] = None,
                via: Optional[List[Tuple[float, float]]] = None) -> int:
        """Walk to (x, y), passing through `via` on the way without stopping
        (a route along a walkway), and optionally end facing `yaw`."""
        return self._begin({"verb": "walk_to", "target": (float(x), float(y)),
                            "target_yaw": None if yaw is None else float(yaw),
                            "via": [(float(a), float(b)) for a, b in (via or [])],
                            "leg": 0, "label": label, "unit": "m"})

    def halt(self) -> int:
        """Stop now: fade the gait out wherever it is and measure nothing
        beyond where the robot came to rest."""
        return self._begin({"verb": "halt"})

    def busy(self) -> bool:
        """A verb is in flight (a finished one stays in `motion` until the
        next order, with `done` set)."""
        m = self.motion
        return m is not None and not m.get("done") and m["verb"] != "stand"

    def moving(self) -> bool:
        return self.blend > 1e-3

    # ── internals ─────────────────────────────────────────────────────

    def _begin(self, m: dict) -> int:
        prev = self.motion
        if prev is not None and prev["verb"] not in ("stand",) and not prev.get("done"):
            # Superseded: a later order took over before this one reported.
            self._finish(prev, superseded=True)
        self.seq += 1
        m["seq"] = self.seq
        m["phase"] = "init"
        m["t0"] = self.sim_time
        self.motion = m
        return self.seq

    def _finish(self, m: dict, **extra) -> None:
        m["done"] = True
        res = {"seq": m["seq"], "verb": m["verb"], "unit": m.get("unit"),
               "commanded": m.get("commanded"),
               "corrections": m.get("corrections", 0),
               "t_start": m.get("t0"), "t_end": self.sim_time,
               "duration_s": self.sim_time - m.get("t0", self.sim_time)}
        res.update(extra)
        self.completed = res

    def _drive(self, vx_cmd: float, wz_cmd: float, on: bool) -> None:
        """Set the target body rates; `on` False fades the gait out."""
        a_vx = self.dt / max(self.VX_TAU_S, self.dt)
        a_wz = self.dt / max(self.WZ_TAU_S, self.dt)
        self.vx += a_vx * (vx_cmd - self.vx)
        self.wz += a_wz * (wz_cmd - self.wz)
        if on:
            if self.blend <= 1e-6:
                # A fresh start from the standing pose: restart the gait
                # clock at the all-feet-planted phase with a zero stride, so
                # the first tick of the gait IS the standing pose.
                self.gait_time = 0.0
                self.phase = _dr.QS_PHASE
            # Ramp up rather than jump: an order that arrives while the
            # previous one is still fading out would otherwise snap every
            # joint from the half-blended pose to the full gait in one tick.
            self.blend = min(1.0, self.blend + self.dt / self.BLEND_UP_S)
        else:
            self.blend = max(0.0, self.blend - self.dt / self.STOP_BLEND_S)
            if self.blend <= 0.0:
                self.vx = self.wz = 0.0

    def _steer_to_line(self, x, y, yaw, m, sign) -> float:
        """Yaw-rate command that holds heading m['yaw0'] on the start line."""
        h0 = m["yaw0"]
        lat = -(x - m["x0"]) * math.sin(h0) + (y - m["y0"]) * math.cos(h0)
        psi_d = h0 - math.atan2(lat, self.LOOKAHEAD_M) * sign
        err = wrap_pi(psi_d - yaw)
        return _clamp(self.K_HEADING * err, -self.wz_max, self.wz_max)

    def _step_motion(self, x: float, y: float, yaw: float) -> Tuple[float, float, bool]:
        """Advance the verb in flight. -> (vx_cmd, wz_cmd, gait_on)."""
        m = self.motion
        if m is None or m.get("done"):
            return 0.0, 0.0, False
        v = m["verb"]
        if m["phase"] == "init":
            m.update(x0=x, y0=y, yaw0=yaw, yaw_prev=yaw, yaw_acc=0.0)
            m["phase"] = "run" if v in ("walk", "turn", "walk_to") else "stop"
        # Unwrapped yaw travelled since the start (a 270 deg turn is 270),
        # and the path length walked.
        m["yaw_acc"] += wrap_pi(yaw - m["yaw_prev"])
        m["yaw_prev"] = yaw
        px, py = m.get("xy_prev", (x, y))
        m["path"] = m.get("path", 0.0) + math.hypot(x - px, y - py)
        m["xy_prev"] = (x, y)

        if m["phase"] == "run":
            if v == "walk":
                cmd = m["commanded"]
                along = (x - m["x0"]) * math.cos(m["yaw0"]) + (y - m["y0"]) * math.sin(m["yaw0"])
                # Direction of travel toward the target from HERE, so a
                # correction after an overshoot walks back.
                sign = 1.0 if cmd - along >= 0 else -1.0
                remaining = abs(cmd - along)
                lead = abs(self.v_meas) * self.lead_walk_s
                if remaining <= max(lead, 0.02):
                    m["phase"] = "stop"
                else:
                    # Slow for the last stretch so the stop lands where asked.
                    speed = self.vx_max * _clamp(remaining / 0.4, 0.35, 1.0)
                    return sign * speed, self._steer_to_line(x, y, yaw, m, sign), True
            elif v == "turn":
                err = m["commanded"] - m["yaw_acc"]
                lead = abs(self.w_meas) * self.lead_turn_s
                if abs(err) <= max(lead, math.radians(1.0)):
                    m["phase"] = "stop"
                else:
                    wz = _clamp(1.5 * err, -self.wz_max, self.wz_max)
                    # Never crawl into the target below a useful rate.
                    if abs(wz) < 0.12:
                        wz = math.copysign(0.12, err)
                    return 0.0, wz, True
            elif v == "walk_to":
                via = m.get("via") or []
                # Pass the via points in order without stopping; the stop is
                # planned only against the final target.
                while m["leg"] < len(via) and math.hypot(
                        via[m["leg"]][0] - x, via[m["leg"]][1] - y) <= self.VIA_PASS_M:
                    m["leg"] += 1
                final = m["leg"] >= len(via)
                tx, ty = m["target"] if final else via[m["leg"]]
                dist = math.hypot(tx - x, ty - y)
                if not final:
                    dist = dist + 1.0            # never start the stop on a via point
                bearing = wrap_pi(math.atan2(ty - y, tx - x) - yaw)
                sub = m.setdefault("sub", "face" if abs(bearing) > self.GOTO_FACE_RAD else "go")
                if sub == "face":
                    lead = abs(self.w_meas) * self.lead_turn_s
                    if abs(bearing) <= max(lead, math.radians(3.0)):
                        m["sub"] = "go"
                    else:
                        wz = _clamp(1.5 * bearing, -self.wz_max, self.wz_max)
                        if abs(wz) < 0.12:
                            wz = math.copysign(0.12, bearing)
                        return 0.0, wz, True
                if m["sub"] == "go":
                    lead = abs(self.v_meas) * self.lead_walk_s
                    if dist <= max(lead, 0.03):
                        if m.get("target_yaw") is not None:
                            m["sub"] = "align"
                        else:
                            m["phase"] = "stop"
                    else:
                        speed = self.vx_max * _clamp(dist / 0.4, 0.35, 1.0)
                        # Walk and steer together; slow down while the heading
                        # error is large so the arc stays tight.
                        speed *= _clamp(1.0 - abs(bearing) / math.radians(45.0), 0.2, 1.0)
                        wz = _clamp(self.K_HEADING * bearing, -self.wz_max, self.wz_max)
                        return speed, wz, True
                if m.get("sub") == "align":
                    err = wrap_pi(m["target_yaw"] - yaw)
                    lead = abs(self.w_meas) * self.lead_turn_s
                    if abs(err) <= max(lead, math.radians(1.0)):
                        m["phase"] = "stop"
                    else:
                        wz = _clamp(1.5 * err, -self.wz_max, self.wz_max)
                        if abs(wz) < 0.12:
                            wz = math.copysign(0.12, err)
                        return 0.0, wz, True

        if m["phase"] == "stop":
            if self.blend > 0.0:
                return 0.0, 0.0, False
            m["phase"] = "settle"
            m["t_settle"] = self.sim_time
        if m["phase"] == "settle":
            if self.sim_time - m["t_settle"] < self.SETTLE_S:
                return 0.0, 0.0, False
            if self._needs_correction(m, x, y, yaw):
                # Same target, measured from the same start: the correction
                # finishes the order that was given, it is not a new one.
                m["corrections"] = m.get("corrections", 0) + 1
                m["phase"] = "run"
                if v == "walk_to":
                    m.pop("sub", None)
                    m["leg"] = len(m.get("via") or [])
                return 0.0, 0.0, False
            self._measure(m, x, y, yaw)
        return 0.0, 0.0, False

    def _needs_correction(self, m: dict, x: float, y: float, yaw: float) -> bool:
        if m.get("corrections", 0) >= self.MAX_CORRECTIONS:
            return False
        v = m["verb"]
        if v == "walk":
            along = (x - m["x0"]) * math.cos(m["yaw0"]) + (y - m["y0"]) * math.sin(m["yaw0"])
            return abs(along - m["commanded"]) > self.WALK_TOL_M
        if v == "turn":
            return abs(m["yaw_acc"] - m["commanded"]) > self.TURN_TOL_RAD
        if v == "walk_to":
            tx, ty = m["target"]
            if math.hypot(tx - x, ty - y) > self.GOTO_TOL_M:
                return True
            ty_ = m.get("target_yaw")
            return ty_ is not None and abs(wrap_pi(yaw - ty_)) > self.TURN_TOL_RAD
        return False

    def _measure(self, m: dict, x: float, y: float, yaw: float) -> None:
        v = m["verb"]
        if v == "walk":
            cmd = m["commanded"]
            along = (x - m["x0"]) * math.cos(m["yaw0"]) + (y - m["y0"]) * math.sin(m["yaw0"])
            lat = -(x - m["x0"]) * math.sin(m["yaw0"]) + (y - m["y0"]) * math.cos(m["yaw0"])
            err = along - cmd
            self._finish(m, achieved=along, error=err, lateral_drift_m=lat,
                         heading_change_rad=m["yaw_acc"],
                         settled=abs(err) <= max(self.WALK_TOL_M, 0.05 * abs(cmd)),
                         pose=[x, y, yaw])
        elif v == "turn":
            cmd = m["commanded"]
            err = m["yaw_acc"] - cmd
            moved = math.hypot(x - m["x0"], y - m["y0"])
            self._finish(m, achieved=m["yaw_acc"], error=err, displacement_m=moved,
                         settled=abs(err) <= self.TURN_TOL_RAD, pose=[x, y, yaw])
        elif v == "walk_to":
            tx, ty = m["target"]
            err = math.hypot(tx - x, ty - y)
            out = {"commanded_xy": [tx, ty], "achieved_xy": [x, y],
                   "achieved_yaw": yaw, "error_m": err, "achieved": None,
                   "arrived": err <= self.GOTO_TOL_M, "tolerance_m": self.GOTO_TOL_M,
                   "start_xy": [m["x0"], m["y0"]], "via": m.get("via") or [],
                   "path_m": m.get("path"), "label": m.get("label"), "pose": [x, y, yaw]}
            ok = err <= self.GOTO_TOL_M
            if m.get("target_yaw") is not None:
                yerr = wrap_pi(yaw - m["target_yaw"])
                out.update(commanded_yaw=m["target_yaw"], yaw_error_rad=yerr)
                ok = ok and abs(yerr) <= self.TURN_TOL_RAD
            out["settled"] = ok
            self._finish(m, **out)
        else:   # halt / stand
            self._finish(m, achieved=None, settled=True, pose=[x, y, yaw])

    RATE_WINDOW_S = 0.5

    def _measure_rates(self, t: float, x: float, y: float, yaw: float) -> None:
        """Body speed and yaw rate differenced over the last RATE_WINDOW_S.
        A crawl surges once per leg swing, so a one-tick derivative is noise."""
        if self._yaw_unwrapped is None:
            self._yaw_unwrapped = yaw
        else:
            self._yaw_unwrapped += wrap_pi(yaw - self._hist[-1][4]) if self._hist else 0.0
        self._hist.append((t, x, y, self._yaw_unwrapped, yaw))
        while len(self._hist) > 2 and t - self._hist[0][0] > self.RATE_WINDOW_S:
            self._hist.pop(0)
        t0, x0, y0, w0, _ = self._hist[0]
        span = t - t0
        if span < 0.1:
            return
        self.v_meas = ((x - x0) * math.cos(yaw) + (y - y0) * math.sin(yaw)) / span
        self.w_meas = (self._yaw_unwrapped - w0) / span

    # Pivot hold: an in-place turn by the yaw sweep alone creeps (measured on
    # the X30 trot: 0.25 m per 90 degrees). While a verb pivots, the stride
    # gains a small body-frame (vx, vy) that pulls it back to where the pivot
    # began. Stride units, like vx: the body delivers about half.
    K_PIVOT_HOLD = 1.5          # 1/s
    PIVOT_HOLD_MAX = 0.15       # m/s of stride

    def _pivot_hold(self, x: float, y: float, yaw: float) -> Optional[Tuple[float, float]]:
        m = self.motion
        if m is None or m.get("done") or m.get("phase") != "run":
            return None
        pivoting = m["verb"] == "turn" or (m["verb"] == "walk_to" and m.get("sub") in ("face", "align"))
        if not pivoting:
            return None
        if m.get("sub") == "align":
            # The final pivot at a place holds the PLACE, not wherever the
            # approach happened to stop: it pulls the robot onto its target
            # while it turns to the requested heading.
            ax, ay = m["target"]
        else:
            ax, ay = m.setdefault("hold_xy_%s" % (m.get("sub") or "turn"), (x, y))
        ex, ey = ax - x, ay - y
        c, s = math.cos(yaw), math.sin(yaw)
        exb, eyb = c * ex + s * ey, -s * ex + c * ey
        lim = self.PIVOT_HOLD_MAX
        return (_clamp(self.K_PIVOT_HOLD * exb, -lim, lim),
                _clamp(self.K_PIVOT_HOLD * eyb, -lim, lim))

    def _add_lateral_stride(self, feet):
        """Add a sideways stride for body rate self.vy: planted feet slide at
        -vy, swinging feet catch up on the same quintic as the forward stride
        (Gait._foot_x_z, transposed to y)."""
        g = self.gait
        Ly = self.vy * g.duty / g.freq
        if g.ramp_s > 0:
            Ly *= min(1.0, max(0.0, self.gait_time / g.ramp_s))
        st = g.stride(self.phase, self.gait_time, self.wz)
        out = {}
        for leg, (fx, fy, fz) in feet.items():
            phi, s = st[leg][2], st[leg][3]
            if s is None:
                off = Ly * (0.5 - phi / g.duty)
            else:
                off = Ly * (s * s * s * (10.0 - 15.0 * s + 6.0 * s * s) - 0.5)
            out[leg] = (fx, fy + off, fz)
        return out

    def tick(self, sim_time: float, x: float, y: float, yaw: float,
             up_body: Optional[Tuple[float, float]] = None) -> Dict[str, Tuple[float, float, float]]:
        """One control step. -> {leg: (hip_x, hip_y, knee)} in the Deep
        Robotics joint convention."""
        self.sim_time = sim_time
        self._measure_rates(sim_time, x, y, yaw)
        vx_cmd, wz_cmd, on = self._step_motion(x, y, yaw)
        vy_cmd = 0.0
        hold = self._pivot_hold(x, y, yaw)
        if hold is not None:
            vx_cmd, vy_cmd = vx_cmd + hold[0], hold[1]
        self._drive(vx_cmd, wz_cmd, on)
        a_vy = self.dt / max(self.VX_TAU_S, self.dt)
        self.vy = 0.0 if self.blend <= 0.0 else self.vy + a_vy * (vy_cmd - self.vy)
        if self.blend <= 0.0:
            return self.stand_q
        # The gait: stride length follows the commanded speed; the clock
        # runs at the gait's own cadence (slower on descents with a map).
        self.gait.vx = self.vx
        speed_factor = getattr(self.crawl, "speed_factor", 1.0) if self.crawl is not None else 1.0
        self.gait_time += self.dt * speed_factor
        self.phase = _dr.QS_PHASE + 2.0 * math.pi * self.gait.freq * self.gait_time
        if self.crawl is not None:
            feet = self.crawl.feet(self.phase, self.gait_time, self.wz, x, y, yaw, up=up_body)
        else:
            feet, _ = self.gait.foot_targets(self.phase, self.gait_time, self.wz)
        if abs(self.vy) > 1e-4:
            feet = self._add_lateral_stride(feet)
        q = self.gait.joints_for(feet)
        b = self.blend
        return {leg: tuple(self.stand_q[leg][i] + b * (q[leg][i] - self.stand_q[leg][i])
                           for i in range(3)) for leg in LEGS}


# ── Self-test: the state machine against an idealised body ─────────────
def _selftest() -> int:
    """Integrates the body from the low-passed commanded rates (a robot that
    does exactly what it is told) and checks every verb ends measured and
    within tolerance. It proves the bookkeeping, not the physics."""
    d = CrawlDriver("x30", dt=0.008)
    x = y = yaw = 0.0
    t = 0.0

    def run(seq_expected, limit_s=120.0):
        nonlocal x, y, yaw, t
        t_end = t + limit_s
        while t < t_end:
            d.tick(t, x, y, yaw)
            x += d.vx * d.blend * math.cos(yaw) * d.dt
            y += d.vx * d.blend * math.sin(yaw) * d.dt
            yaw = wrap_pi(yaw + d.wz * d.blend * d.dt)
            t += d.dt
            c = d.completed
            if c is not None and c["seq"] == seq_expected:
                return c
        raise AssertionError("motion %d never completed" % seq_expected)

    ok = True
    for label, issue, check in (
        ("walk 2 m", lambda: d.walk(2.0), lambda c: abs(c["error"]) < 0.15),
        ("turn +90", lambda: d.turn(math.pi / 2), lambda c: abs(c["error"]) < math.radians(8)),
        ("walk -0.5 m", lambda: d.walk(-0.5), lambda c: abs(c["error"]) < 0.15),
        ("turn -135", lambda: d.turn(-3 * math.pi / 4), lambda c: abs(c["error"]) < math.radians(8)),
        ("walk_to (1, 3) facing 0", lambda: d.walk_to(1.0, 3.0, 0.0), lambda c: c["error_m"] < 0.25),
        ("route via (3,3),(3,0)", lambda: d.walk_to(0.0, 0.0, math.pi, via=[(3.0, 3.0), (3.0, 0.0)]),
         lambda c: c["error_m"] < 0.25 and c["path_m"] > 5.0),
    ):
        c = run(issue())
        good = bool(check(c))
        ok &= good
        print("%-24s %s  %s" % (label, "OK " if good else "BAD",
                                {k: (round(v, 3) if isinstance(v, float) else v)
                                 for k, v in c.items() if k in (
                                     "achieved", "error", "error_m", "settled", "duration_s",
                                     "path_m")}))
    return 0 if ok else 1


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
