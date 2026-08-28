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
"""RoboLife robot brain -- PURE Python (no ``omnisim`` import, no engine).

Everything a RoboLife robot decides lives here so it can be unit-tested
without a simulator: the supervisor bus parser, diff-drive kinematics, the
docking geometry, the go-to / lidar-guard control laws and the state machine.
``controllers/robolife_robot/robolife_robot.py`` is the thin engine-facing
shell that feeds ``Brain.step`` measured state and applies its output.

Conventions (from projects/robolife/DESIGN.md):

* World is ENU, yaw about +z, robot forward is +x of its own frame.
* The robot carries two Connectors on its x axis: ``socket_front`` at
  +SOCKET_OFFSET facing +x and ``socket_rear`` at -SOCKET_OFFSET facing -x.
* A module carries one passive ``plug`` on its -x face at height 0.25
  facing -x. The bus reports the module's CENTRE and BODY yaw, so the plug's
  normal points ``module_yaw + pi`` and sits ``MODULE_HALF_X[type]`` from the
  centre along that normal (``module_plug_pose``).
* A plug mates with the FRONT socket when the robot faces ``plug_yaw + pi``
  (nose into the plug) and with the REAR socket when the robot faces
  ``plug_yaw`` (backing in). ``approach_pose`` returns the robot-centre pose
  that puts the chosen socket ``standoff`` metres out along the plug normal.
* Lidar range images run LEFT to RIGHT (index 0 = +fov/2 = the robot's left,
  Webots convention, verified by ``lidar_wgpu_azimuth_smoke``). Non-returns
  are inf/nan/0 and are ignored.
* Positive ``w`` is a left (counter-clockwise) turn.

MEASURE-THEN-CORRECT. The Husky is a skid-steer and under Newton a commanded
yaw rate delivers only a fraction of itself (the shipped mobile bridge carried
a 56.7% turn error for months because it trusted the command). The brain
therefore never assumes its yaw command lands: it learns ``w_gain`` =
achieved / commanded from the measured yaw rate the controller feeds it and
divides its desired rate by that gain. All heading control is closed-loop on
the MEASURED pose; the gain only sets how hard it pushes.
"""
from __future__ import annotations

import copy
import json
import math
import random
from typing import Dict, List, Optional, Sequence, Tuple

# ------------------------------------------------------------------ constants
WHEEL_RADIUS = 0.1651        # m, Husky (DESIGN.md)
TRACK = 0.555                # m, Husky wheel separation (DESIGN.md)
SOCKET_OFFSET = 0.55         # m, both sockets sit on the x axis at +-0.55
WHEEL_MAX_RADPS = 7.5        # saturation for wheel commands (~1.24 m/s)
W_MAX = 1.5                  # rad/s, ceiling on the desired body yaw rate
STANDOFF = 0.9               # m, socket-to-plug distance of the align pose
CREEP_SPEED = 0.15           # m/s, straight-in docking speed
CREEP_GRACE_S = 3.0          # s past the expected mate time before backing off
DOCK_RETRIES = 1             # extra align->creep attempts after the first
BACKOFF_M = 0.6              # m the robot backs away before a retry
BLACKLIST_S = 30.0           # s a module that failed to dock is ignored
RELEASE_COOLDOWN_S = 20.0    # s a module we were ORDERED to drop is ignored
CLEAR_M = 0.6                # m backed away from a just-released module
CLEAR_SPEED = 0.3            # m/s for that manoeuvre
BLOCKED_ESCAPE_S = 4.0       # s continuously blocked before reversing out
ESCAPE_S = 2.5               # s of straight reverse in the escape
MODULE_LOST_S = 4.0          # s a seek target may be missing from the bus
CHARGE_FULL = 0.9            # leave `charging` at this fraction
PAD_ARRIVE_M = 0.5           # aim to be this close to the pad centre (<0.9)
WAYPOINT_TIMEOUT_S = 25.0
ALIGN_POS_TOL = 0.08         # m (= the Connector distanceTolerance)
ALIGN_YAW_TOL = 0.06         # rad
SEEK_HANDOFF_M = 0.35        # seek_module -> dock when this close to align pose
FRONT_HALF_DEG = 40.0        # lidar guard window is +-40 deg
W_GAIN_INIT = 0.5
W_GAIN_MIN, W_GAIN_MAX = 0.08, 1.5
W_GAIN_ALPHA = 0.05
W_GAIN_MIN_CMD = 0.25        # rad/s, only learn from clearly non-zero commands
W_GAIN_STEADY_S = 0.5        # s the command must be steady before a sample

STATES = ("explore", "seek_module", "dock", "seek_charge", "charging", "stopped")
MODULE_TYPES = ("battery", "solar", "mast", "armor")

# Half x-extent of each module body (DESIGN.md table; rl/modules.py [A] is
# the authority for geometry -- keep these in step with it).
MODULE_HALF_X = {"battery": 0.17, "solar": 0.20, "mast": 0.05, "armor": 0.18}

DEFAULT_GENOME = {
    "cruise_speed": 0.7,
    "charge_at": 0.3,
    "module_pref": {"battery": 1.0, "solar": 1.0, "mast": 1.0, "armor": 1.0},
    "greed": 0.5,
    "caution": 1.0,
    "explore_radius": 8.0,
}

DEFAULT_BUS = {
    "t": 0.0,
    "batt": 1.0,
    "cap_wh": 200.0,
    "state_hint": "ok",
    "pads": [],
    "bay": [0.0, 0.0],
    "modules": [],
    "orders": [],
    "genome": DEFAULT_GENOME,
}


# ------------------------------------------------------------------ helpers
def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _f(x, default: float) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return default
    if math.isnan(v) or math.isinf(v):
        return default
    return v


def _xy(p):
    try:
        return [float(p[0]), float(p[1])]
    except (TypeError, ValueError, IndexError, KeyError):
        return None


# ------------------------------------------------------------------ bus
def parse_genome(g) -> dict:
    """Merge a (possibly partial / garbage) genome onto DEFAULT_GENOME."""
    out = copy.deepcopy(DEFAULT_GENOME)
    if not isinstance(g, dict):
        return out
    for k in ("cruise_speed", "charge_at", "greed", "caution", "explore_radius"):
        if k in g:
            out[k] = _f(g[k], out[k])
    out["cruise_speed"] = clamp(out["cruise_speed"], 0.05, 2.0)
    out["charge_at"] = clamp(out["charge_at"], 0.0, 0.95)
    out["greed"] = clamp(out["greed"], 0.0, 1.0)
    out["caution"] = clamp(out["caution"], 0.2, 5.0)
    out["explore_radius"] = clamp(out["explore_radius"], 1.0, 100.0)
    pref = g.get("module_pref")
    if isinstance(pref, dict):
        for k in MODULE_TYPES:
            if k in pref:
                out["module_pref"][k] = max(0.0, _f(pref[k], out["module_pref"][k]))
    return out


def parse_bus(custom_data) -> dict:
    """Parse the supervisor->robot customData JSON. Tolerant: an empty string,
    non-JSON, a non-object, or any malformed member yields the defaults for
    that member; the result always carries every DESIGN.md key."""
    out = copy.deepcopy(DEFAULT_BUS)
    if not custom_data:
        return out
    try:
        raw = (json.loads(custom_data) if isinstance(custom_data, (str, bytes))
               else custom_data)
    except (ValueError, TypeError):
        return out
    if not isinstance(raw, dict):
        return out
    out["t"] = _f(raw.get("t"), 0.0)
    out["batt"] = clamp(_f(raw.get("batt"), 1.0), 0.0, 1.0)
    out["cap_wh"] = max(0.0, _f(raw.get("cap_wh"), 200.0))
    hint = raw.get("state_hint")
    out["state_hint"] = hint if hint in ("ok", "low", "dead") else "ok"
    pads = raw.get("pads")
    if isinstance(pads, list):
        out["pads"] = [p for p in (_xy(q) for q in pads) if p is not None]
    bay = _xy(raw.get("bay"))
    if bay is not None:
        out["bay"] = bay
    mods = raw.get("modules")
    if isinstance(mods, list):
        clean = []
        for m in mods:
            if not isinstance(m, dict) or "id" not in m:
                continue
            try:
                mid = int(m["id"])
            except (TypeError, ValueError):
                continue
            mtype = m.get("type")
            if mtype not in MODULE_TYPES:
                mtype = "battery" if mtype is None else str(mtype)
            clean.append({
                "id": mid, "type": mtype,
                "x": _f(m.get("x"), 0.0), "y": _f(m.get("y"), 0.0),
                "yaw": _f(m.get("yaw"), 0.0),
                "loose": bool(m.get("loose", True)),
            })
        out["modules"] = clean
    orders = raw.get("orders")
    if isinstance(orders, str):
        orders = [orders]
    if isinstance(orders, list):
        out["orders"] = [o for o in orders if isinstance(o, str)]
    out["genome"] = parse_genome(raw.get("genome"))
    return out


# ------------------------------------------------------------------ kinematics
def diff_drive(v: float, w: float, wheel_radius: float = WHEEL_RADIUS,
               track: float = TRACK) -> Tuple[float, float]:
    """Body twist (v m/s, w rad/s) -> (left, right) wheel speeds in rad/s."""
    half = 0.5 * track
    return (v - w * half) / wheel_radius, (v + w * half) / wheel_radius


def wheel_speeds_to_twist(left: float, right: float, wheel_radius: float = WHEEL_RADIUS,
                          track: float = TRACK) -> Tuple[float, float]:
    """Inverse of ``diff_drive``: measured wheel rad/s -> (v, w)."""
    v = 0.5 * wheel_radius * (left + right)
    w = wheel_radius * (right - left) / track
    return v, w


def saturate_wheels(left: float, right: float, max_radps: float = WHEEL_MAX_RADPS
                    ) -> Tuple[float, float]:
    """Scale both wheels by one factor so neither exceeds ``max_radps``;
    the v:w ratio (the arc) is preserved."""
    big = max(abs(left), abs(right))
    if big <= max_radps or big == 0.0:
        return left, right
    s = max_radps / big
    return left * s, right * s


# ------------------------------------------------------------------ geometry
def module_plug_pose(module: dict) -> Tuple[float, float, float]:
    """(plug_x, plug_y, plug_yaw) of a module from its bus record (centre +
    body yaw). The plug is on the -x face, so its normal is yaw + pi."""
    yaw = float(module.get("yaw", 0.0))
    half = MODULE_HALF_X.get(module.get("type"), 0.17)
    plug_yaw = wrap_pi(yaw + math.pi)
    return (float(module["x"]) + half * math.cos(plug_yaw),
            float(module["y"]) + half * math.sin(plug_yaw),
            plug_yaw)


def approach_pose(plug_x: float, plug_y: float, plug_yaw: float, standoff: float,
                  socket: str = "front", socket_offset: float = SOCKET_OFFSET
                  ) -> Tuple[float, float, float]:
    """Robot-centre (x, y, yaw) that puts the chosen socket ``standoff`` metres
    out along the plug normal, aimed so a straight creep along the normal
    mates. Front socket: robot faces ``plug_yaw + pi`` (nose to the plug).
    Rear socket: robot faces ``plug_yaw`` (backs in). In both cases the
    centre is ``standoff + socket_offset`` along the normal."""
    nx, ny = math.cos(plug_yaw), math.sin(plug_yaw)
    d = standoff + socket_offset
    yaw = wrap_pi(plug_yaw + math.pi) if socket == "front" else wrap_pi(plug_yaw)
    return plug_x + nx * d, plug_y + ny * d, yaw


def socket_position(pose: Sequence[float], socket: str = "front",
                    socket_offset: float = SOCKET_OFFSET) -> Tuple[float, float]:
    """World xy of a socket given the robot-centre pose."""
    x, y, yaw = pose[0], pose[1], pose[2]
    s = socket_offset if socket == "front" else -socket_offset
    return x + s * math.cos(yaw), y + s * math.sin(yaw)


def heading_error(pose: Sequence[float], target: Sequence[float]) -> float:
    """Signed bearing error (rad) from the pose's yaw to the target point;
    positive = target is to the left."""
    bearing = math.atan2(target[1] - pose[1], target[0] - pose[0])
    return wrap_pi(bearing - pose[2])


def dist2d(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ------------------------------------------------------------------ control laws
def go_to(pose: Sequence[float], target: Sequence[float], cruise: float,
          caution_stop: Optional[float] = None, k_turn: float = 2.0,
          slow_radius: float = 1.2, stop_radius: float = 0.15,
          w_max: float = W_MAX) -> Tuple[float, float]:
    """(v, w) toward ``target`` = (x, y) or (x, y, yaw).

    * turns in place while the bearing error is large, blends into forward
      motion as it shrinks, and slows linearly inside ``slow_radius``;
    * inside ``stop_radius`` it stops, then (if a final yaw is given) rotates
      in place to it;
    * ``caution_stop`` is the measured free distance straight ahead (metres,
      from ``lidar_front``); when given, v is capped so it reaches zero at
      0.3 m of clearance. None = no obstacle information.
    """
    d = dist2d(pose, target)
    if d <= stop_radius:
        if len(target) >= 3 and target[2] is not None:
            err = wrap_pi(float(target[2]) - pose[2])
            if abs(err) > 0.02:
                return 0.0, clamp(k_turn * err, -w_max, w_max)
        return 0.0, 0.0
    err = heading_error(pose, target)
    w = clamp(k_turn * err, -w_max, w_max)
    # Forward speed: zero beyond 60 deg of error, full when aligned.
    align = clamp(1.0 - abs(err) / math.radians(60.0), 0.0, 1.0)
    v = cruise * align * clamp(d / slow_radius, 0.25, 1.0)
    if caution_stop is not None:
        v = min(v, max(0.0, 0.6 * (caution_stop - 0.3)))
    return v, w


def _valid_range(r) -> bool:
    try:
        return r > 0.0 and not math.isinf(r) and not math.isnan(r) and r < 1e6
    except TypeError:
        return False


def lidar_min(ranges: Optional[Sequence[float]]) -> Optional[float]:
    """Smallest valid return over the whole scan, or None."""
    if not ranges:
        return None
    best = None
    for r in ranges:
        if _valid_range(r) and (best is None or r < best):
            best = r
    return best


def lidar_front(ranges: Optional[Sequence[float]], fov: float,
                front_half_deg: float = FRONT_HALF_DEG
                ) -> Tuple[Optional[float], Optional[float], float, float]:
    """Nearest return inside the front +-``front_half_deg`` window.
    Returns (nearest_m, nearest_angle_rad, mean_free_left, mean_free_right);
    angles follow the lidar convention (+ = left)."""
    if not ranges:
        return None, None, float("inf"), float("inf")
    n = len(ranges)
    if n < 2 or fov <= 0:
        return None, None, float("inf"), float("inf")
    step = fov / (n - 1)
    lim = math.radians(front_half_deg)
    best, best_th = None, None
    left_sum, left_n, right_sum, right_n = 0.0, 0, 0.0, 0
    for i, r in enumerate(ranges):
        th = 0.5 * fov - i * step
        if abs(th) > lim:
            continue
        val = r if _valid_range(r) else None
        if val is not None and (best is None or val < best):
            best, best_th = val, th
        free = val if val is not None else 1e3
        if th >= 0:
            left_sum += free
            left_n += 1
        else:
            right_sum += free
            right_n += 1
    ml = left_sum / left_n if left_n else float("inf")
    mr = right_sum / right_n if right_n else float("inf")
    return best, best_th, ml, mr


def lidar_guard(ranges: Optional[Sequence[float]], fov: float, caution_m: float,
                front_half_deg: float = FRONT_HALF_DEG) -> Tuple[bool, float]:
    """(blocked, steer_bias) from a planar scan.

    ``blocked`` is True when the nearest return inside the front
    +-``front_half_deg`` window is closer than ``caution_m``. ``steer_bias``
    is in [-1, 1], positive = turn LEFT (positive w): an obstacle on the
    left biases right and vice versa, magnitude growing as it nears; a
    dead-ahead obstacle steers toward whichever half of the window is freer.
    (False, 0.0) when nothing is in range or the scan is empty."""
    nearest, th, ml, mr = lidar_front(ranges, fov, front_half_deg)
    if nearest is None or nearest >= caution_m:
        return False, 0.0
    mag = clamp(1.0 - nearest / caution_m, 0.15, 1.0)
    if abs(th) < math.radians(3.0):
        return True, (1.0 if ml >= mr else -1.0)
    return True, (-mag if th > 0 else mag)


# ------------------------------------------------------------------ choice
def choose_module(modules: List[dict], pose: Sequence[float], genome: dict,
                  docked: dict, blacklist: Dict[int, float], t: float
                  ) -> Tuple[Optional[dict], Optional[str]]:
    """argmax of module_pref[type] * greed / distance over loose modules that
    are not blacklisted or already ours. Returns (module, socket) -- the
    socket is the front if free, else the rear; (None, None) when no socket
    is free, greed is zero, or nothing scores > 0."""
    socket = "front" if docked.get("front") is None else (
        "rear" if docked.get("rear") is None else None)
    if socket is None:
        return None, None
    greed = float(genome.get("greed", 0.0))
    if greed <= 0.0:
        return None, None
    pref = genome.get("module_pref", {})
    mine = {docked.get("front"), docked.get("rear")}
    best, best_s = None, 0.0
    for m in modules:
        if not m.get("loose", True) or m["id"] in mine:
            continue
        if blacklist.get(m["id"], -1.0) > t:
            continue
        weight = float(pref.get(m["type"], 0.0))
        if weight <= 0.0:
            continue
        d = max(dist2d(pose, (m["x"], m["y"])), 0.5)
        s = weight * greed / d
        if s > best_s:
            best, best_s = m, s
    return (best, socket) if best is not None else (None, None)


def nearest_pad(pads: List[Sequence[float]], pose: Sequence[float]) -> Optional[List[float]]:
    if not pads:
        return None
    return min(pads, key=lambda p: dist2d(pose, p))


# ------------------------------------------------------------------ the brain
class Brain:
    """RoboLife robot state machine.

    ``step(bus, own_pose, own_vel, lidar, presence_front, presence_rear, t)``
    -> ``{"v", "w", "lock_front", "lock_rear", "unlock_front", "unlock_rear",
    "state", "target", "phase", "note"}``. ``own_pose`` = (x, y, yaw) and
    ``own_vel`` = (v, w) are MEASURED by the caller (GPS / IMU deltas, never
    the previous command); ``lidar`` = (ranges, fov) or None; ``t`` is sim
    seconds. ``note`` is a human-readable transition line or None. The
    returned ``v``/``w`` are what the wheels were asked for AFTER saturation
    and the yaw-gain correction, so the caller can log commanded vs achieved.

    States: explore -> seek_module -> dock -> explore; seek_charge when
    batt < genome.charge_at (pads known); charging until >= CHARGE_FULL;
    stopped while the bus orders "stop" or reports state_hint "dead".
    """

    def __init__(self, slot: int = 0, genome: Optional[dict] = None,
                 seed: Optional[int] = None, standoff: float = STANDOFF,
                 creep_speed: float = CREEP_SPEED, origin: Sequence[float] = (0.0, 0.0)):
        self.slot = int(slot)
        self.genome = parse_genome(genome)
        self.rng = random.Random(self.slot * 7919 + 17 if seed is None else seed)
        self.standoff = float(standoff)
        self.creep_speed = float(creep_speed)
        self.origin = (float(origin[0]), float(origin[1]))
        self.state = "explore"
        self.target: Optional[int] = None
        self.docked: Dict[str, Optional[int]] = {"front": None, "rear": None}
        self.blacklist: Dict[int, float] = {}
        self.w_gain = W_GAIN_INIT
        self._w_cmd_prev = 0.0
        self._w_steady_since: Optional[float] = None
        self._steady_sign = 1.0
        self._waypoint: Optional[List[float]] = None
        self._wp_t0 = 0.0
        self._seek_socket = "front"
        self._seek_last_seen = 0.0
        self._dock: dict = {}
        self._clear: Optional[dict] = None      # back-away after a release
        self._avoid_dir = 0.0                   # committed guard side while blocked
        self._blocked_since: Optional[float] = None
        self._escape_until: Optional[float] = None
        self._last_t: Optional[float] = None
        self.last_out: dict = {}

    # -- public -----------------------------------------------------------
    def step(self, bus, own_pose: Sequence[float], own_vel: Sequence[float],
             lidar, presence_front: int, presence_rear: int, t: float) -> dict:
        bus = bus if isinstance(bus, dict) else parse_bus(bus)
        if "genome" in bus:
            self.genome = parse_genome(bus["genome"])
        pose = (float(own_pose[0]), float(own_pose[1]), wrap_pi(float(own_pose[2])))
        w_meas = float(own_vel[1])
        self._learn_w_gain(w_meas, t)
        notes: List[str] = []
        out = {"v": 0.0, "w": 0.0, "lock_front": False, "lock_rear": False,
               "unlock_front": False, "unlock_rear": False}

        # 1. orders -------------------------------------------------------
        orders = bus.get("orders") or []
        for socket, sign in (("front", -1.0), ("rear", 1.0)):
            if "release_" + socket in orders and self.docked[socket] is not None:
                mid = self.docked[socket]
                out["unlock_" + socket] = True
                notes.append("order release_%s -> unlock module %s (ignored %.0f s, backing %.1f m)"
                             % (socket, mid, RELEASE_COOLDOWN_S, CLEAR_M))
                self.docked[socket] = None
                # A supervisor-ordered drop must not be undone on the next tick:
                # the module is sitting at the socket with presence 1.
                self.blacklist[mid] = t + RELEASE_COOLDOWN_S
                self._clear = {"sign": sign, "x0": pose, "until": t + CLEAR_M / CLEAR_SPEED + 1.0}
                if self.state in ("seek_module", "dock") and self.target == mid:
                    self._enter("explore", notes, "released the module being sought")
        halted = ("stop" in orders) or bus.get("state_hint") == "dead"
        if halted and self.state != "stopped":
            self._enter("stopped", notes, "order stop / dead")
        elif not halted and self.state == "stopped":
            self._enter("explore", notes, "stop lifted")

        # 2. battery ------------------------------------------------------
        batt = float(bus.get("batt", 1.0))
        pads = bus.get("pads") or []
        if (self.state not in ("stopped", "seek_charge", "charging")
                and batt < self.genome["charge_at"] and pads):
            self._enter("seek_charge", notes, "batt %.2f < charge_at %.2f"
                        % (batt, self.genome["charge_at"]))

        # 3. dispatch -----------------------------------------------------
        v_des, w_des = 0.0, 0.0
        clearing = False
        if self._clear is not None and self.state not in ("stopped", "charging"):
            c = self._clear
            if t < c["until"] and dist2d(pose, c["x0"]) < CLEAR_M:
                clearing = True
                v_des = c["sign"] * CLEAR_SPEED
            else:
                self._clear = None
        if clearing:
            pass
        elif self.state == "stopped":
            pass
        elif self.state == "charging":
            if batt >= CHARGE_FULL:
                self._enter("explore", notes, "charged %.2f" % batt)
        elif self.state == "seek_charge":
            v_des, w_des = self._seek_charge(pose, lidar, pads, notes, t)
        elif self.state == "explore":
            v_des, w_des = self._explore(bus, pose, lidar, t, notes)
        elif self.state == "seek_module":
            v_des, w_des = self._seek_module(bus, pose, lidar, t, notes)
        elif self.state == "dock":
            v_des, w_des = self._dock_step(bus, pose, presence_front, presence_rear,
                                           t, out, notes)

        # 4. actuate with the learned yaw gain --------------------------
        w_cmd = w_des / max(self.w_gain, W_GAIN_MIN)
        left, right = saturate_wheels(*diff_drive(v_des, w_cmd))
        v_cmd, w_cmd = wheel_speeds_to_twist(left, right)
        self._w_cmd_prev = w_cmd
        self._last_t = t
        out.update({"v": v_cmd, "w": w_cmd, "state": self.state, "target": self.target,
                    "phase": self._dock.get("phase") if self.state == "dock" else None,
                    "note": "; ".join(notes) if notes else None})
        self.last_out = out
        return out

    # -- sub-behaviours ----------------------------------------------------
    def _enter(self, state: str, notes: List[str], why: str) -> None:
        if state not in STATES:
            raise ValueError(state)
        notes.append("%s -> %s (%s)" % (self.state, state, why))
        self.state = state
        if state in ("explore", "stopped", "seek_charge", "charging"):
            self.target = None
            self._dock = {}
        if state == "explore":
            self._waypoint = None

    def _guarded(self, v, w, lidar, caution, t) -> Tuple[float, float]:
        """Apply the lidar guard to a (v, w) intent.

        The avoidance SIDE is committed for as long as the scan stays blocked
        (a dead-ahead obstacle otherwise flips the "freer side" choice every
        tick and the robot dithers in place -- measured on the fake rig:
        +-1.8 rad/s alternating, net rotation zero, stuck for 90 s). Blocked
        continuously for BLOCKED_ESCAPE_S -> reverse straight for ESCAPE_S."""
        if self._escape_until is not None:
            if t < self._escape_until:
                return -CLEAR_SPEED, 0.0
            self._escape_until = None
        if lidar is None:
            return v, w
        ranges, fov = lidar
        blocked, bias = lidar_guard(ranges, fov, caution)
        if not blocked:
            self._avoid_dir = 0.0
            self._blocked_since = None
            return v, w
        if self._blocked_since is None:
            self._blocked_since = t
        elif t - self._blocked_since > BLOCKED_ESCAPE_S:
            self._blocked_since = None
            self._avoid_dir = 0.0
            self._escape_until = t + ESCAPE_S
            return -CLEAR_SPEED, 0.0
        if self._avoid_dir == 0.0:
            self._avoid_dir = 1.0 if bias >= 0 else -1.0
        bias = self._avoid_dir * abs(bias)
        nearest, _, _, _ = lidar_front(ranges, fov)
        clearance = nearest if nearest is not None else caution
        v = min(v, max(0.0, 0.6 * (clearance - 0.3)))
        w = clamp(w + bias * W_MAX, -W_MAX, W_MAX)
        if clearance < 0.45:            # too close to arc around: pivot only
            v = 0.0
            w = self._avoid_dir * W_MAX
        return v, w

    def _explore(self, bus, pose, lidar, t, notes):
        m, socket = choose_module(bus.get("modules") or [], pose, self.genome,
                                  self.docked, self.blacklist, t)
        if m is not None:
            self._seek_socket = socket
            self._seek_last_seen = t
            self._enter("seek_module", notes, "module %d %s -> %s socket"
                        % (m["id"], m["type"], socket))
            self.target = m["id"]
            return 0.0, 0.0
        if (self._waypoint is None or dist2d(pose, self._waypoint) < 0.5
                or t - self._wp_t0 > WAYPOINT_TIMEOUT_S):
            self._waypoint = self._new_waypoint(bus)
            self._wp_t0 = t
        v, w = go_to(pose, self._waypoint, self.genome["cruise_speed"])
        return self._guarded(v, w, lidar, self.genome["caution"], t)

    def _new_waypoint(self, bus) -> List[float]:
        r = self.genome["explore_radius"]
        cx, cy = self.origin
        bay = bus.get("bay")
        if bay:
            cx, cy = float(bay[0]), float(bay[1])
        ang = self.rng.uniform(-math.pi, math.pi)
        rad = r * math.sqrt(self.rng.uniform(0.15, 1.0))
        return [cx + rad * math.cos(ang), cy + rad * math.sin(ang)]

    @staticmethod
    def _find_module(bus, mid) -> Optional[dict]:
        for m in bus.get("modules") or []:
            if m["id"] == mid:
                return m
        return None

    def _seek_module(self, bus, pose, lidar, t, notes):
        m = self._find_module(bus, self.target)
        if m is None:
            if t - self._seek_last_seen > MODULE_LOST_S:
                self._enter("explore", notes, "module %s lost" % self.target)
            return self._guarded(0.0, 0.0, lidar, self.genome["caution"], t)
        self._seek_last_seen = t
        if not m.get("loose", True):
            self._enter("explore", notes, "module %s no longer loose" % self.target)
            return 0.0, 0.0
        px, py, pyaw = module_plug_pose(m)
        ax, ay, ayaw = approach_pose(px, py, pyaw, self.standoff, self._seek_socket)
        if dist2d(pose, (ax, ay)) < SEEK_HANDOFF_M:
            self._dock = {"phase": "align", "socket": self._seek_socket, "module": m["id"],
                          "approach": (ax, ay, ayaw), "plug": (px, py, pyaw),
                          "attempts": 0, "t0": t, "x0": pose}
            self._enter("dock", notes, "align on module %d" % m["id"])
            return 0.0, 0.0
        v, w = go_to(pose, (ax, ay), self.genome["cruise_speed"])
        caution = self.genome["caution"]
        if dist2d(pose, (m["x"], m["y"])) < 2.5:
            caution = min(caution, 0.4)   # the module itself is ahead now
        return self._guarded(v, w, lidar, caution, t)

    def _dock_step(self, bus, pose, presence_front, presence_rear, t, out, notes):
        d = self._dock
        socket = d["socket"]
        sign = 1.0 if socket == "front" else -1.0
        presence = presence_front if socket == "front" else presence_rear
        m = self._find_module(bus, d["module"])
        if m is not None and m.get("loose", True):
            # re-derive the geometry every tick: the module may have been nudged
            px, py, pyaw = module_plug_pose(m)
            d["plug"] = (px, py, pyaw)
            d["approach"] = approach_pose(px, py, pyaw, self.standoff, socket)
        elif m is not None:
            self._enter("explore", notes, "module %d taken during dock" % d["module"])
            return 0.0, 0.0
        ax, ay, ayaw = d["approach"]
        phase = d["phase"]

        if phase == "align":
            v, w = go_to(pose, (ax, ay, ayaw), min(self.genome["cruise_speed"], 0.5),
                         stop_radius=ALIGN_POS_TOL)
            if (dist2d(pose, (ax, ay)) <= ALIGN_POS_TOL
                    and abs(wrap_pi(ayaw - pose[2])) <= ALIGN_YAW_TOL):
                d.update(phase="creep", t0=t, x0=pose)
                notes.append("dock: aligned (pos err %.3f m, yaw err %.3f rad) -> creep"
                             % (dist2d(pose, (ax, ay)), wrap_pi(ayaw - pose[2])))
                return 0.0, 0.0
            return v, w

        if phase == "creep":
            if presence:
                out["lock_" + socket] = True
                self.docked[socket] = d["module"]
                sx, sy = socket_position(pose, socket)
                gap = dist2d((sx, sy), d["plug"])
                notes.append("dock: presence on %s -> lock module %d (socket-plug gap %.3f m, "
                             "creep %.1f s, attempt %d)" % (socket, d["module"], gap,
                                                             t - d["t0"], d["attempts"] + 1))
                self._enter("explore", notes, "docked %s" % socket)
                return 0.0, 0.0
            expected = self.standoff / max(self.creep_speed, 1e-3)
            travelled = dist2d(pose, d["x0"])
            if t - d["t0"] > expected + CREEP_GRACE_S or travelled > self.standoff + 0.3:
                notes.append("dock: no presence after %.1f s / %.2f m -> back off"
                             % (t - d["t0"], travelled))
                d.update(phase="backoff", t0=t, x0=pose)
                return 0.0, 0.0
            # Hold the approach yaw AND steer the socket onto the plug's normal
            # line: the connector mates only inside distanceTolerance (0.08 m),
            # so a heading-only creep from a 0.1 m lateral miss sails past.
            sx, sy = socket_position(pose, socket)
            px, py = d["plug"][0], d["plug"][1]
            lateral = -(sx - px) * math.sin(pose[2]) + (sy - py) * math.cos(pose[2])
            w = 2.0 * wrap_pi(ayaw - pose[2]) - sign * 3.0 * lateral
            return sign * self.creep_speed, clamp(w, -0.4, 0.4)

        if phase == "backoff":
            travelled = dist2d(pose, d["x0"])
            if travelled >= BACKOFF_M or t - d["t0"] > 6.0:
                d["attempts"] += 1
                if d["attempts"] <= DOCK_RETRIES:
                    d.update(phase="align", t0=t)
                    notes.append("dock: retry %d on module %d" % (d["attempts"], d["module"]))
                    return 0.0, 0.0
                self.blacklist[d["module"]] = t + BLACKLIST_S
                self._enter("explore", notes,
                            "dock on module %d failed %d times; blacklisted %.0f s"
                            % (d["module"], d["attempts"], BLACKLIST_S))
                return 0.0, 0.0
            return -sign * 2.0 * self.creep_speed, 0.0

        raise RuntimeError("unknown dock phase %r" % phase)

    def _seek_charge(self, pose, lidar, pads, notes, t):
        pad = nearest_pad(pads, pose)
        if pad is None:
            self._enter("explore", notes, "no pad known")
            return 0.0, 0.0
        if dist2d(pose, pad) <= PAD_ARRIVE_M:
            self._enter("charging", notes, "on pad (%.2f, %.2f)" % (pad[0], pad[1]))
            return 0.0, 0.0
        v, w = go_to(pose, pad, self.genome["cruise_speed"], stop_radius=PAD_ARRIVE_M * 0.8)
        return self._guarded(v, w, lidar, self.genome["caution"], t)

    # -- yaw-rate gain ---------------------------------------------------
    def _learn_w_gain(self, w_meas: float, t: float) -> None:
        """Update w_gain = achieved / commanded from the MEASURED yaw rate,
        sampling only after the previous command has been steady (same sign,
        |w| >= W_GAIN_MIN_CMD) for W_GAIN_STEADY_S so spin-up lag does not
        bias it."""
        wc = self._w_cmd_prev
        if abs(wc) < W_GAIN_MIN_CMD:
            self._w_steady_since = None
            return
        sign = 1.0 if wc > 0 else -1.0
        if self._w_steady_since is None or sign != self._steady_sign:
            self._w_steady_since = t
            self._steady_sign = sign
            return
        if t - self._w_steady_since < W_GAIN_STEADY_S:
            return
        ratio = clamp(w_meas / wc, W_GAIN_MIN, W_GAIN_MAX)
        self.w_gain = clamp((1.0 - W_GAIN_ALPHA) * self.w_gain + W_GAIN_ALPHA * ratio,
                            W_GAIN_MIN, W_GAIN_MAX)

    # -- status --------------------------------------------------------
    def status(self, v_meas: float, w_meas: float, lidar_min_m: Optional[float]) -> dict:
        """The robot->supervisor bus record, exactly DESIGN.md's shape. v/w
        are the caller's MEASURED values -- never the command."""
        return {
            "state": self.state,
            "v": round(float(v_meas), 4),
            "w": round(float(w_meas), 4),
            "docked": {"front": self.docked["front"], "rear": self.docked["rear"]},
            "target": self.target,
            "lidar_min": None if lidar_min_m is None else round(float(lidar_min_m), 3),
        }
