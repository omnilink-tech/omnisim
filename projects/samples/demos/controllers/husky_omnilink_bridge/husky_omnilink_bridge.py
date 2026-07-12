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

"""Husky OmniLink bridge - supervisor controller for the husky_maze world.

Optional Webots devices (auto-detected at startup):
    front_camera    - Camera node attached to the URDFRobot's children.
                      Enables /camera, /capabilities.camera_available,
                      and the agent's read_camera tool. Vision-only worlds
                      depend on this; if the camera is missing the bridge
                      runs without it.



Mirrors the design of `omnilink_arm_bridge`: a Webots controller that
owns the Husky's motors and pose, and exposes a small HTTP surface so an
external OmniLink agent can drive the robot. The agent never imports the
Webots `controller` module - it talks JSON over HTTP, this bridge
translates into wheel-motor commands.

Wired into the world: husky_maze.wbt's Husky has supervisor=TRUE and
controller="husky_omnilink_bridge". The controller runs inside the
Husky's process, so `Supervisor.getSelf()` returns the Husky node and we
can read its world pose every tick.

HTTP surface (default port 6070, loopback only):

    GET  /state
        {
          x, y, yaw,                  # world frame, meters and radians
          v_linear, v_angular,        # measured from successive poses
          left_speed, right_speed,    # commanded wheel velocities (rad/s)
          mode,                       # idle | velocity | goto_cell | stopped
          fault,                      # null or short string
          sim_time,                   # seconds
          last_tick_at,               # wall-clock epoch seconds
          target,                     # nullable: {col, row, x, y} for goto
          goal_reached                # true when within GOAL_RADIUS_M of (10, -10)
        }

    GET  /capabilities
        {
          robot_id, model,
          wheel_motors: [...],
          max_wheel_speed_radps,
          wheel_radius_m,
          half_track_m,
          maze: {
            cell_size_m, cols, rows,
            origin_x, origin_y,        # world coords of cell (0, 0)
            start: {col, row, x, y},
            goal:  {col, row, x, y},
            goal_radius_m,
          }
        }

    POST /action  {"action": "...", ...params}
        stop                        - zero both wheels, mode -> stopped
        reset                       - teleport Husky back to start cell
        set_velocity                - {linear, angular} m/s, rad/s; clamped
        drive_forward               - {distance, speed?}; closed-loop on pose
        turn                        - {angle, speed?}; closed-loop on yaw
        goto_cell                   - {col, row, speed?}; pure-pursuit to cell
                                      center along a straight line (no maze
                                      planning - the agent picks safe cells)

The bridge does primitive motion only. Maze planning lives in the agent
prompt + knowledge files.
"""

from __future__ import annotations

import base64
import io
import json
import math
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from omnisim import Supervisor


# Mirror all stdout to a log file so we can read the controller's prints
# from outside Webots — Webots' GUI-only console is unreachable from the
# tool-driven dev loop.
class _Tee:
    def __init__(self, *streams):
        self._streams = streams
    def write(self, s):
        for st in self._streams:
            try: st.write(s); st.flush()
            except Exception: pass
    def flush(self):
        for st in self._streams:
            try: st.flush()
            except Exception: pass

import os as _os_for_log
import tempfile as _tempfile_for_log
import sys as _sys_for_log
_log_path = _os_for_log.path.join(_tempfile_for_log.gettempdir(),
                                  f"husky_omnilink_bridge_{_os_for_log.getpid()}.log")
try:
    _log_fp = open(_log_path, "w", encoding="utf-8", buffering=1)
    _sys_for_log.stdout = _Tee(_sys_for_log.__stdout__, _log_fp)
    _sys_for_log.stderr = _Tee(_sys_for_log.__stderr__, _log_fp)
    print(f"[husky_omnilink_bridge] log file: {_log_path}")
except Exception as _e:
    pass


# --- Husky kinematics (from clearpath URDF) ---------------------------------

WHEEL_RADIUS_M = 0.1651
HALF_TRACK_M = 0.2854   # half the lateral wheel separation
MAX_WHEEL_SPEED = 6.0   # rad/s, matches husky_random.py

WHEEL_MOTORS = (
    "front_left_wheel_motor",
    "rear_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_right_wheel_motor",
)

# --- Maze layout (matches husky_maze.wbt header comments) -------------------

MAZE = {
    "cell_size_m": 2.0,
    "cols": 11,
    "rows": 11,
    # cell (col, row) center -> world (-10 + 2*col, -10 + 2*row)
    "origin_x": -10.0,
    "origin_y": -10.0,
    "start": {"col": 0, "row": 10, "x": -10.0, "y": 10.0},
    "goal":  {"col": 10, "row": 0, "x": 10.0, "y": -10.0},
    "goal_radius_m": 0.8,
}

# --- Bridge config ----------------------------------------------------------

BRIDGE_HOST = "127.0.0.1"
# Default 6070 / 6071 preserves the single-husky world contract that all
# pre-multiplexing demos rely on (warehouse_logistics, husky_maze_*).
# Worlds with multiple huskies override per-instance via Webots
# controllerArgs:
#   controller "husky_omnilink_bridge"
#   controllerArgs ["--port" "6080" "--eye-port" "6081"]
# Order doesn't matter; --port and --eye-port can appear in either
# position. Anything else in controllerArgs is silently ignored so
# wrapper scripts can pass extra flags without breaking us.
import argparse as _argparse


def _parse_bridge_args() -> tuple:
    parser = _argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", type=int, default=6070)
    parser.add_argument("--eye-port", dest="eye_port", type=int, default=6071)
    args, _unknown = parser.parse_known_args()
    return args.port, args.eye_port


BRIDGE_PORT, EYE_SIDECAR_PORT = _parse_bridge_args()

# Closed-loop controller gains.
HEADING_KP = 3.0          # rad/s of spin per rad of heading error
ALIGN_TOL_RAD = 0.05      # turn loop done when heading error inside this
TURN_FIRST_RAD = 0.10     # in goto_cell, stop and spin if heading error > this
                          # (tightened 0.15 -> 0.10: smaller residual heading
                          # error at drive-phase entry = less corner skid)
GOTO_REACH_TOL_M = 0.18   # cell considered reached when within this
GOTO_TIMEOUT_S = 120.0    # safety abort. 120 s sim is deliberately
                          # generous: the wheel-only no-snap controller
                          # needs to actually pivot and drive without
                          # cheating, and under heavy CPU contention
                          # (4-way RL training in parallel) sim_time
                          # advances slowly relative to controller torque
                          # ramp-up. 120 s comfortably covers worst-case
                          # 90° pivot + 2 m drive even at 1/4 speed.
# Pivot angular speed is capped below MAX_ANGULAR_R_S so the wheels
# don't break traction during in-place 90° turns. 1.5 rad/s = ~1.0 s sim
# for a 90° pivot under nominal load, vs ~0.45 s at full speed which
# tends to skid the body sideways and wedge against the next wall.
PIVOT_ANGULAR_R_S_MAX = 1.5
# Wheel command ramping. Original tuning was "effectively unlimited" (6.0)
# because Webots' built-in motor torque limits smooth the diff-drive
# transitions. Keep that: testing showed lower ramp values made the husky
# arrive at the same wedge in the same sim time but slower wall time,
# without improving the wedge rate. Slow pivots come from PIVOT_ANGULAR_R_S_MAX,
# not from the wheel ramp.
WHEEL_RAMP_PER_TICK = 6.0
MAX_LINEAR_M_S = MAX_WHEEL_SPEED * WHEEL_RADIUS_M     # ~0.99 m/s
MAX_ANGULAR_R_S = MAX_WHEEL_SPEED * WHEEL_RADIUS_M / HALF_TRACK_M  # ~3.47 rad/s


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_orientation(orient) -> float:
    if orient is None or len(orient) < 9:
        return 0.0
    return math.atan2(orient[3], orient[0])


def find_link_by_name(node, target):
    """Walk a URDFRobot subtree for a Solid with the given name."""
    if node is None:
        return None
    name_field = node.getField("name")
    if name_field is not None:
        try:
            if name_field.getSFString() == target:
                return node
        except Exception:
            pass
    children = node.getField("children")
    if children is not None:
        try:
            count = children.getCount()
        except Exception:
            count = 0
        for i in range(count):
            found = find_link_by_name(children.getMFNode(i), target)
            if found is not None:
                return found
    endpoint = node.getField("endPoint")
    if endpoint is not None:
        found = find_link_by_name(endpoint.getSFNode(), target)
        if found is not None:
            return found
    return None


_NON_POSE_TYPES = {"Camera", "Lidar", "RangeFinder", "DistanceSensor",
                   "GPS", "Compass", "Accelerometer", "Gyro", "Receiver",
                   "Emitter", "LED", "Display", "Speaker", "Microphone",
                   "InertialUnit", "TouchSensor", "Pen", "PointLight",
                   "SpotLight", "DirectionalLight", "Group", "Pose"}


def find_first_physical_child(node, depth: int = 0):
    """Best-effort walk for a child Solid whose getPosition is finite.

    Used when find_link_by_name doesn't turn up a known link name —
    we still want SOMETHING with a real world position to drive off.
    Skip sensor / actuator nodes whose getPosition is the LOCAL offset
    of the device (e.g. a Camera whose .translation is its mounting
    offset relative to the parent), not the body's world pose."""
    if node is None or depth > 6:
        return None
    try:
        type_name = node.getTypeName()
    except Exception:
        type_name = ""
    skip = type_name in _NON_POSE_TYPES
    if not skip:
        try:
            pos = node.getPosition()
            if pos and len(pos) >= 2 and not (math.isnan(pos[0]) or math.isnan(pos[1])):
                if depth > 0:  # skip the URDFRobot wrapper itself
                    return node
        except Exception:
            pass
    children = node.getField("children")
    if children is not None:
        try:
            count = children.getCount()
        except Exception:
            count = 0
        for i in range(count):
            child = children.getMFNode(i)
            found = find_first_physical_child(child, depth + 1)
            if found is not None:
                return found
    endpoint = node.getField("endPoint")
    if endpoint is not None:
        found = find_first_physical_child(endpoint.getSFNode(), depth + 1)
        if found is not None:
            return found
    return None


def encode_png(width: int, height: int, bgra_bytes: bytes) -> bytes:
    """Encode a Webots camera image (BGRA) as a PNG byte string.

    Pure-Python — no PIL dependency. Matches Webots' image layout: rows
    top-to-bottom, 4 bytes per pixel (B, G, R, A). We re-pack to RGB and
    emit a single IDAT chunk."""
    # BGRA -> RGB. Drop alpha; reorder.
    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        b = bgra_bytes[i * 4 + 0]
        g = bgra_bytes[i * 4 + 1]
        r = bgra_bytes[i * 4 + 2]
        rgb[i * 3 + 0] = r
        rgb[i * 3 + 1] = g
        rgb[i * 3 + 2] = b

    # PNG: signature + IHDR + IDAT + IEND.
    sig = b"\x89PNG\r\n\x1a\n"

    # IHDR
    ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    ihdr = _png_chunk(b"IHDR", ihdr_data)

    # IDAT — prepend each scanline with filter byte 0 (None), then deflate.
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * stride:(y + 1) * stride])
    idat = _png_chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    iend = _png_chunk(b"IEND", b"")
    return sig + ihdr + idat + iend


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


def _probe_camera():
    """Quick reachability probe for the husky_eye sidecar (default port
    6071, configurable per-instance via the bridge's --eye-port arg)."""
    try:
        import urllib.request as _u
        with _u.urlopen(f"http://127.0.0.1:{EYE_SIDECAR_PORT}/status", timeout=0.3) as r:
            s = json.loads(r.read().decode("utf-8", errors="replace"))
        return {
            "kind": "robot_camera",
            "source": f"husky_eye sidecar @127.0.0.1:{EYE_SIDECAR_PORT}",
            "ready": bool(s.get("ready")),
            "width": s.get("width"),
            "height": s.get("height"),
            "fov_rad": s.get("fov_rad"),
            "encoding": "image/png; base64",
        }
    except Exception:
        return {
            "kind": "operator_viewport",
            "source": "Supervisor.exportImage (fallback)",
            "ready": False,
            "encoding": "image/png; base64",
            "note": (
                "husky_eye sidecar not reachable — agent will get a "
                "diagnostic operator-viewport frame. Vision-only worlds "
                "must include a husky_eye Robot."
            ),
        }


def diff_drive_to_wheel_speeds(linear_m_s: float, angular_r_s: float):
    """Convert {linear, angular} body twist to {left, right} wheel rad/s.

    Both wheels on the same side share a velocity (skid-steer)."""
    v_left = (linear_m_s - angular_r_s * HALF_TRACK_M) / WHEEL_RADIUS_M
    v_right = (linear_m_s + angular_r_s * HALF_TRACK_M) / WHEEL_RADIUS_M
    return v_left, v_right


# ---------------------------------------------------------------------------
# Scene queries — the bridge sees the whole world; the agent only sees what
# we expose as tool surfaces. This is what makes "the agent doesn't know
# the map" honest: we never ship the wall list out of the bridge unless the
# operator (or the world's title) says it's OK to.
# ---------------------------------------------------------------------------

LIDAR_NUM_RAYS = 16
LIDAR_MAX_RANGE_M = 8.0
# Lidar starts above the wheels so it doesn't graze the floor.
LIDAR_Z_OFFSET = 0.4


def collect_walls(supervisor):
    """Walk the world for Wall nodes; return AABBs in the world XY plane.

    Walls are upright boxes — we only care about (cx, cy, sx, sy). All
    walls in this maze are axis-aligned with rotation = identity, so their
    world AABB is just (cx ± sx/2, cy ± sy/2)."""
    root = supervisor.getRoot()
    children_field = root.getField("children") if root is not None else None
    walls = []
    if children_field is None:
        return walls
    try:
        count = children_field.getCount()
    except Exception:
        return walls
    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            type_name = node.getTypeName()
        except Exception:
            continue
        if type_name != "Wall":
            continue
        try:
            translation = node.getField("translation").getSFVec3f()
            size = node.getField("size").getSFVec3f()
            name = node.getField("name").getSFString()
        except Exception:
            continue
        walls.append({
            "name": name,
            "cx": float(translation[0]),
            "cy": float(translation[1]),
            "sx": float(size[0]),
            "sy": float(size[1]),
        })
    return walls


def ray_aabb_distance(ox, oy, dx, dy, w, max_range):
    """Distance along ray (ox,oy)+t*(dx,dy) until it hits axis-aligned
    box `w`, or None if no hit within (0, max_range].

    Standard slab method, simplified to 2D since walls are tall vertical
    boxes and we never tilt the lidar."""
    minx = w["cx"] - w["sx"] * 0.5
    maxx = w["cx"] + w["sx"] * 0.5
    miny = w["cy"] - w["sy"] * 0.5
    maxy = w["cy"] + w["sy"] * 0.5

    inv_dx = 1.0 / dx if abs(dx) > 1e-12 else float("inf")
    inv_dy = 1.0 / dy if abs(dy) > 1e-12 else float("inf")

    tx1 = (minx - ox) * inv_dx
    tx2 = (maxx - ox) * inv_dx
    if tx1 > tx2:
        tx1, tx2 = tx2, tx1

    ty1 = (miny - oy) * inv_dy
    ty2 = (maxy - oy) * inv_dy
    if ty1 > ty2:
        ty1, ty2 = ty2, ty1

    t_enter = max(tx1, ty1)
    t_exit = min(tx2, ty2)
    if t_exit < 0 or t_enter > t_exit:
        return None
    if t_enter < 1e-6:
        # Ray origin is inside the box (shouldn't happen unless the husky
        # is wedged inside a wall) — report exit distance so the agent at
        # least sees something.
        if t_exit > max_range:
            return None
        return t_exit
    if t_enter > max_range:
        return None
    return t_enter


def lidar_scan(x, y, yaw, walls, num_rays=LIDAR_NUM_RAYS, max_range=LIDAR_MAX_RANGE_M):
    """Cast num_rays evenly around the husky and return per-ray range.

    Angles are reported in the BODY frame: 0 = forward, +pi/2 = left,
    -pi/2 = right, +/-pi = back. World-frame ray angle for ray i is
    yaw + body_angle[i]."""
    body_angles = [(2.0 * math.pi * i / num_rays) - math.pi for i in range(num_rays)]
    ranges = []
    hits = []
    for ba in body_angles:
        wa = wrap_pi(yaw + ba)
        dx = math.cos(wa)
        dy = math.sin(wa)
        nearest = max_range
        nearest_wall = None
        for w in walls:
            t = ray_aabb_distance(x, y, dx, dy, w, max_range)
            if t is not None and t < nearest:
                nearest = t
                nearest_wall = w["name"]
        ranges.append(nearest)
        hits.append(nearest_wall)
    return body_angles, ranges, hits


def build_maze_graph_from_walls(walls):
    """Mirror of agents/production/husky_maze/maze.py — derive the cell
    adjacency graph from a list of (cx, cy, sx, sy) walls. Used when the
    operator allows the bridge to reveal the map."""
    cells = MAZE["cols"]
    blocked = set()
    for w in walls:
        sx = w["sx"]
        sy = w["sy"]
        cx = w["cx"]
        cy = w["cy"]
        # H wall: 2 m along x, thin along y -> blocks N-S passage.
        if abs(sx - 2.0) < 0.05 and abs(sy - 0.2) < 0.05:
            col = round((cx - MAZE["origin_x"]) / MAZE["cell_size_m"])
            row_lo = round((cy - MAZE["origin_y"] - 1.0) / MAZE["cell_size_m"])
            if 0 <= col < cells and 0 <= row_lo < cells - 1:
                blocked.add(((col, row_lo), (col, row_lo + 1)))
                blocked.add(((col, row_lo + 1), (col, row_lo)))
        # V wall: thin along x, 2 m along y -> blocks E-W passage.
        elif abs(sx - 0.2) < 0.05 and abs(sy - 2.0) < 0.05:
            col_lo = round((cx - MAZE["origin_x"] - 1.0) / MAZE["cell_size_m"])
            row = round((cy - MAZE["origin_y"]) / MAZE["cell_size_m"])
            if 0 <= col_lo < cells - 1 and 0 <= row < cells:
                blocked.add(((col_lo, row), (col_lo + 1, row)))
                blocked.add(((col_lo + 1, row), (col_lo, row)))
        # Otherwise: a perimeter wall (size 22.2 x 0.2) or something else
        # we don't model in the cell graph.

    adj = {}
    for col in range(cells):
        for row in range(cells):
            here = (col, row)
            nbs = []
            for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nc, nr = col + dc, row + dr
                if not (0 <= nc < cells and 0 <= nr < cells):
                    continue
                if (here, (nc, nr)) in blocked:
                    continue
                nbs.append([nc, nr])
            adj[f"{col},{row}"] = nbs
    return adj


def build_blocked_edges(walls):
    """Return the set of blocked (cell_a, cell_b) edge tuples. Cheaper than
    build_maze_graph_from_walls when the caller only needs to test legality
    of a single hop. Symmetric: both (a,b) and (b,a) are inserted."""
    cells = MAZE["cols"]
    blocked = set()
    for w in walls:
        sx, sy, cx, cy = w["sx"], w["sy"], w["cx"], w["cy"]
        if abs(sx - 2.0) < 0.05 and abs(sy - 0.2) < 0.05:
            col = round((cx - MAZE["origin_x"]) / MAZE["cell_size_m"])
            row_lo = round((cy - MAZE["origin_y"] - 1.0) / MAZE["cell_size_m"])
            if 0 <= col < cells and 0 <= row_lo < cells - 1:
                blocked.add(((col, row_lo), (col, row_lo + 1)))
                blocked.add(((col, row_lo + 1), (col, row_lo)))
        elif abs(sx - 0.2) < 0.05 and abs(sy - 2.0) < 0.05:
            col_lo = round((cx - MAZE["origin_x"] - 1.0) / MAZE["cell_size_m"])
            row = round((cy - MAZE["origin_y"]) / MAZE["cell_size_m"])
            if 0 <= col_lo < cells - 1 and 0 <= row < cells:
                blocked.add(((col_lo, row), (col_lo + 1, row)))
                blocked.add(((col_lo + 1, row), (col_lo, row)))
    return blocked


def is_legal_hop(walls, prev_cr, curr_cr):
    """True iff curr_cr is a 4-cardinal neighbour of prev_cr with no wall
    between them. Used to refuse `goto_cell` / `execute_path` calls that
    would otherwise lean on the wedge-recovery snap to clip walls."""
    pc, pr = prev_cr
    cc, cr = curr_cr
    if (pc, pr) == (cc, cr):
        return True  # zero-hop is trivially legal
    if abs(pc - cc) + abs(pr - cr) != 1:
        return False  # not a 4-cardinal neighbour
    return ((pc, pr), (cc, cr)) not in build_blocked_edges(walls)


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_linear = 0.0
        self.v_angular = 0.0
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.mode = "idle"
        self.fault = None
        self.sim_time = 0.0
        self.last_tick_at = time.time()
        self.tick_period_s = 0.016
        # World metadata, populated at startup and never mutated. The agent
        # only sees `world_title` and the gated /maze response — the actual
        # walls list stays bridge-local so the agent can't cheat.
        self.world_title = ""
        self.reveal_map = True
        # Blind-mode flag: when the world title contains "blind", /lidar is
        # gated to {available: false}. Combined with /maze gating ("unknown"
        # in title), this gives us the layered discriminator stack: layer 1
        # = strategy choice (BFS vs lidar wall-follow), layer 2 = brief
        # interpretation, layer 3 = vision-only navigation with no map AND
        # no lidar — pure pixels + pose.
        self.reveal_lidar = True
        self.walls = []  # list of {name, cx, cy, sx, sy}
        # Mission brief — free-form natural language read from WorldInfo.info.
        # The brief tells the operator's intent in human terms; the agent
        # must interpret it and decide when it's complete by calling
        # the `complete_mission` action. The bridge tracks completion in
        # `mission_complete` and audits each claim in `mission_log` so a
        # script that drives the bridge directly cannot pretend to satisfy
        # an arbitrary brief — only an LLM can interpret intent.
        self.mission_brief = ""
        self.mission_complete = False
        self.mission_log = []  # list of {timestamp, rationale, claimed_cells, sim_time}
        # Optional Webots Camera device — left unused; see vision_path
        # below for the actual approach.
        self.camera = None
        self.camera_width = 0
        self.camera_height = 0
        self.camera_fov_rad = 0.0
        # Vision capture via Supervisor.exportImage. The HTTP handler
        # increments vision_request; the main loop notices, exports a PNG
        # to vision_path, and bumps vision_served. The handler then reads
        # the file and returns it. This indirection keeps Supervisor calls
        # on the main thread, which is what Webots requires.
        self.vision_path = None  # absolute path to the output PNG (set in main)
        self.vision_width = 800
        self.vision_height = 600
        self.vision_request = 0
        self.vision_served = 0
        # Active task (set by handlers; consumed by main loop).
        # task is one of None | {kind: "velocity", linear, angular}
        #                      | {kind: "drive_forward", target_x, target_y, speed, deadline}
        #                      | {kind: "turn", target_yaw, speed, deadline}
        #                      | {kind: "goto_cell", col, row, x, y, speed, deadline}
        self.task = None
        # `reset_request` set to a dict {x, y, yaw} by /action reset and consumed
        # by the main loop, which is the only place allowed to touch the scene.
        self.reset_request = None
        # `reload_request` set to "" (current world) or a filename by
        # /admin/reload. Consumed by the main loop. Triggers worldReload /
        # worldLoad, which respawns this controller process. Used by tests
        # so the harness doesn't have to reach into the Webots window.
        self.reload_request = None
        self.goal_reached = False
        # Visited-cells set, populated each tick from the husky's pose.
        # Vision-only mazes (no map, no lidar) need episodic memory or
        # the agent will loop on the same dead-end indefinitely. Storing
        # this server-side is more reliable than asking the LLM to keep
        # a mental list across turns.
        self.visited_cells = set()

    def snapshot(self):
        with self.lock:
            target = None
            if self.task and self.task.get("kind") == "goto_cell":
                target = {
                    "col": self.task["col"],
                    "row": self.task["row"],
                    "x": self.task["x"],
                    "y": self.task["y"],
                }
            # Pre-compute the husky's current cell server-side. The agent
            # was tripping on its own col/row arithmetic in the blind-world
            # vision protocol — for pose (-2.25, 6.01) the LLM kept rounding
            # 7.75/2 = 3.875 down to 3 and snapping back to cell (3, 8) when
            # actual cell is (4, 8), creating a mini-loop. Surfacing
            # current_cell directly removes the agent from that math.
            cur_col = int(round((self.x - MAZE["origin_x"]) / MAZE["cell_size_m"]))
            cur_row = int(round((self.y - MAZE["origin_y"]) / MAZE["cell_size_m"]))
            cell_x = MAZE["origin_x"] + cur_col * MAZE["cell_size_m"]
            cell_y = MAZE["origin_y"] + cur_row * MAZE["cell_size_m"]
            return {
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "v_linear": self.v_linear,
                "v_angular": self.v_angular,
                "left_speed": self.left_cmd,
                "right_speed": self.right_cmd,
                "mode": self.mode,
                "fault": self.fault,
                "sim_time": self.sim_time,
                "last_tick_at": self.last_tick_at,
                "target": target,
                "goal_reached": self.goal_reached,        # legacy: hardcoded (10,0)
                "mission_complete": self.mission_complete, # agent-set via complete_mission
                # current_cell: world-pose -> grid-cell mapping done server-
                # side. drift_m is the husky's distance from the cell centre
                # — the agent can use it to decide whether a snap_to_cell is
                # warranted.
                "current_cell": {
                    "col": cur_col,
                    "row": cur_row,
                    "x": cell_x,
                    "y": cell_y,
                    "drift_m": ((self.x - cell_x) ** 2 + (self.y - cell_y) ** 2) ** 0.5,
                },
                # Visited-cells episodic memory + unvisited-cardinal-neighbour
                # hint. The vision-only protocol on husky_maze_blind.wbt
                # without this lands the agent in a dead-end loop because it
                # has no recall of where it has been. visited_cells is the
                # set of cells the husky has actually settled in
                # (drift < cell_size/3); unvisited_neighbours is the list of
                # 4-connected cardinal neighbours of current_cell that are
                # in-grid AND not in visited_cells. The agent should prefer
                # an unvisited_neighbour when the camera shows multiple open
                # corridors.
                "visited_cells": sorted(list(self.visited_cells)),
                "unvisited_neighbours": [
                    [n_col, n_row] for n_col, n_row in [
                        (cur_col + 1, cur_row),  # east
                        (cur_col - 1, cur_row),  # west
                        (cur_col, cur_row + 1),  # north
                        (cur_col, cur_row - 1),  # south
                    ]
                    if 0 <= n_col < MAZE["cols"]
                    and 0 <= n_row < MAZE["rows"]
                    and (n_col, n_row) not in self.visited_cells
                ],
            }


# ---------------------------------------------------------------------------
# Synchronous-mode helper
# ---------------------------------------------------------------------------

def _wait_for_task_idle(state: "BridgeState", timeout_s: float, poll_s: float = 0.01):
    """Block on the HTTP handler thread until the physics loop has cleared
    state.task (success or fault), or until the wall-clock deadline expires.
    Used by goto_cell?wait=true and execute_path so the agent gets one
    response per move instead of having to poll get_state. Returns
    {done, fault, x, y, yaw, sim_time}.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        with state.lock:
            t = state.task
            f = state.fault
            x, y, yaw, st = state.x, state.y, state.yaw, state.sim_time
        if t is None:
            return {"done": True, "fault": f, "x": x, "y": y, "yaw": yaw, "sim_time": st}
        time.sleep(poll_s)
    with state.lock:
        f = state.fault
        x, y, yaw, st = state.x, state.y, state.yaw, state.sim_time
    return {"done": False, "fault": f or "http_wait_timeout", "x": x, "y": y, "yaw": yaw, "sim_time": st}


# ---------------------------------------------------------------------------
# Cardinal-aware perception + motion helpers (used by walk_one_cell /
# follow_corridor). The agent navigates in *world* cardinals (north / south
# / east / west) and the bridge translates to body-frame motion. This frees
# the LLM from yaw arithmetic entirely.
# ---------------------------------------------------------------------------

WORLD_CARDINALS = ("east", "north", "west", "south")
CARDINAL_TO_YAW = {"east": 0.0, "north": math.pi / 2, "west": math.pi, "south": -math.pi / 2}
CARDINAL_DELTAS = {"east": (1, 0), "west": (-1, 0), "north": (0, 1), "south": (0, -1)}
# Body-frame camera direction relative to the husky's facing cardinal,
# expressed in CCW quarter-rotations: front=0, left=+1 (90° CCW),
# back=+2, right=-1 (90° CW).
_BODY_QUARTER_OFFSET = {"front": 0, "left": 1, "back": 2, "right": -1}


def _yaw_to_cardinal_index(yaw: float) -> int:
    """Snap yaw (rad) to the nearest world cardinal index in WORLD_CARDINALS."""
    return int(round(yaw / (math.pi / 2))) % 4


def _body_to_world_cardinal(body_cam: str, yaw: float) -> str:
    """Map a body-frame camera ('front'/'right'/'back'/'left') to the world
    cardinal it is currently pointing at, given the husky's yaw."""
    base = _yaw_to_cardinal_index(yaw)
    return WORLD_CARDINALS[(base + _BODY_QUARTER_OFFSET[body_cam]) % 4]


def _fetch_eye_scan() -> dict | None:
    """Fetch the eye sidecar's /scan output. Returns the parsed dict on
    success, None on failure (sidecar absent / timeout)."""
    try:
        import urllib.request as _u
        req = _u.Request(f"http://127.0.0.1:{EYE_SIDECAR_PORT}/scan", method="GET")
        with _u.urlopen(req, timeout=5) as r:
            return json.loads(r.read().decode("utf-8", errors="replace"))
    except Exception:
        return None


def _make_scan_digest(eye_scan, yaw, current_cell_cr, visited_cells_iter):
    """Compress the eye's per-cardinal analysis into a small map keyed by
    *world* cardinal. The LLM consumes this in place of the raw 4×9-field
    scan dump, dropping per-turn input from ~1.3 KB to ~250 bytes and
    removing per-cardinal score/std arithmetic from the prompt.

    Each cardinal becomes one of {open, blocked, ambiguous}:
      - blocked   : wall_close=true OR score >= 0.6
      - ambiguous : score in [0.3, 0.6) — recommend read_camera fallback
      - open      : score < 0.3

    Marker is the strongest coloured-cylinder hit across all four cardinals
    with `world_cardinal` resolved (so the agent knows e.g. 'red is north',
    not 'red is on the right camera which depends on my yaw')."""
    cams = (eye_scan or {}).get("cameras", {}) or {}
    by_world: dict[str, str] = {}
    marker = None
    for body_name in ("front", "right", "back", "left"):
        c = cams.get(body_name) or {}
        wc = _body_to_world_cardinal(body_name, yaw)
        score = float(c.get("wall_close_score", 0.0) or 0.0)
        wall = bool(c.get("wall_close", False))
        if wall or score >= 0.6:
            tag = "blocked"
        elif score >= 0.3:
            tag = "ambiguous"
        else:
            tag = "open"
        by_world[wc] = tag
        m_color = c.get("marker")
        try:
            m_frac = float(c.get("marker_fraction") or 0.0)
        except Exception:
            m_frac = 0.0
        if m_color and m_frac >= 0.005 and (marker is None or m_frac > marker.get("fraction", 0.0)):
            cent = c.get("marker_centroid") or {}
            marker = {
                "color": m_color,
                "world_cardinal": wc,
                "fraction": round(m_frac, 4),
                "centroid_x_norm": (round(float(cent.get("x_norm", 0.5)), 2) if cent else None),
                # Hints for the agent:
                #   approach_recommended = visible AND not blocked → drive that way
                #   adjacent             = visible AND blocked     → marker is in the next cell
                "approach_recommended": (not wall) and m_frac > 0.05,
                "adjacent": wall and m_frac > 0.20,
            }
    cur_col, cur_row = (current_cell_cr[0], current_cell_cr[1]) if current_cell_cr else (None, None)
    visited_set = {(int(c[0]), int(c[1])) for c in (visited_cells_iter or [])}
    open_cards = [c for c, t in by_world.items() if t == "open"]
    blocked_cards = [c for c, t in by_world.items() if t == "blocked"]
    ambig_cards = [c for c, t in by_world.items() if t == "ambiguous"]
    unvisited_open = []
    if cur_col is not None:
        for c in open_cards:
            dc, dr = CARDINAL_DELTAS[c]
            tc, tr = cur_col + dc, cur_row + dr
            if (0 <= tc < MAZE["cols"] and 0 <= tr < MAZE["rows"]
                    and (tc, tr) not in visited_set):
                unvisited_open.append(c)
    return {
        "current_cell": [cur_col, cur_row] if cur_col is not None else None,
        "facing": WORLD_CARDINALS[_yaw_to_cardinal_index(yaw)],
        "open": sorted(open_cards),
        "blocked": sorted(blocked_cards),
        "ambiguous": sorted(ambig_cards),
        "unvisited_open": sorted(unvisited_open),
        "marker": marker,
    }


def _walk_one_cell_inline(state, cardinal: str, speed_frac: float,
                          safety_scan: bool = True):
    """Pre-snap to the current cell facing `cardinal`, drive 2 m forward,
    snap to destination cell. Returns a dict the action handlers wrap into
    JSON. Refuses (without driving) when the eye-sidecar scan flags the
    target cardinal as wall_close. Used by both walk_one_cell and the
    inner loop of follow_corridor — keeps logic in one place.

    `safety_scan=False` skips the pre-flight wall_close check. Use only
    when the caller has just verified this cardinal is open (e.g.
    follow_corridor's iteration N+1 trusting iteration N's post-walk
    digest). Skipping avoids a known eye-sidecar race where a back-to-back
    /scan call right after a snap returns a stale frame from before the
    eye teleport propagated to the renderer."""
    if cardinal not in CARDINAL_TO_YAW:
        return {"ok": False, "fault": "bad_cardinal", "reason": f"cardinal must be one of {list(CARDINAL_TO_YAW)}"}
    target_yaw = CARDINAL_TO_YAW[cardinal]
    speed = clamp(float(speed_frac) * MAX_LINEAR_M_S, 0.05, MAX_LINEAR_M_S)
    with state.lock:
        cur_x, cur_y, cur_yaw = state.x, state.y, state.yaw
    cur_col = int(round((cur_x - MAZE["origin_x"]) / MAZE["cell_size_m"]))
    cur_row = int(round((cur_y - MAZE["origin_y"]) / MAZE["cell_size_m"]))
    dc, dr = CARDINAL_DELTAS[cardinal]
    tcol, trow = cur_col + dc, cur_row + dr
    if not (0 <= tcol < MAZE["cols"] and 0 <= trow < MAZE["rows"]):
        return {
            "ok": False, "fault": "off_grid",
            "reason": f"target cell ({tcol},{trow}) outside grid",
            "from_cell": [cur_col, cur_row],
        }
    # Safety scan was here historically but is REMOVED:
    #   1. After a failed drive (drive_forward_timeout), the husky drifts
    #      slightly into the wall it hit. On the next walk_one_cell call,
    #      the safety scan from the wedged pose sees the wall on every
    #      cardinal and refuses every direction — agent gets stuck.
    #   2. The eye sidecar's capture has an inherent ~1-tick lag after
    #      the husky teleports. Back-to-back safety scans race the lag
    #      and produce false-positive wall_close readings even when the
    #      previous post-walk scan said the cardinal was open.
    # Trust drive_forward's own timeout as the wall-hit detector. The
    # cost of one wasted ~3-second timeout per misjudged direction is
    # less than the cost of being permanently stuck refusing safe moves.
    # safety_scan parameter retained for callers that want pre-flight
    # checks but is now a no-op until the eye-lag race is fixed.
    _ = safety_scan
    # Drive to the target cell via the wheel-driven goto_cell controller
    # (turn-in-place to face the cell, then forward). This replaces the
    # former teleport pre-snap + drive_forward + drift-snap pattern;
    # teleport-snapping has been removed globally, so the husky now moves
    # entirely under wheel control and is left wherever the wheels settle.
    tx = MAZE["origin_x"] + tcol * MAZE["cell_size_m"]
    ty = MAZE["origin_y"] + trow * MAZE["cell_size_m"]
    with state.lock:
        state.task = {
            "kind": "goto_cell",
            "col": tcol, "row": trow,
            "x": tx, "y": ty,
            "speed": speed,
            "deadline": state.sim_time + GOTO_TIMEOUT_S,
        }
        state.mode = "goto_cell"
        state.fault = None
    res = _wait_for_task_idle(state, GOTO_TIMEOUT_S + 5.0)
    fx, fy, fyaw = res["x"], res["y"], res["yaw"]
    fault = res["fault"]
    eye_after = _fetch_eye_scan()
    with state.lock:
        visited_now = list(state.visited_cells)
    final_col = int(round((fx - MAZE["origin_x"]) / MAZE["cell_size_m"]))
    final_row = int(round((fy - MAZE["origin_y"]) / MAZE["cell_size_m"]))
    digest = _make_scan_digest(eye_after, fyaw, [final_col, final_row], visited_now) if eye_after else None
    return {
        "ok": fault is None,
        "fault": fault,
        "from_cell": [cur_col, cur_row],
        "to_cell": [final_col, final_row],
        "final_pose": {"x": fx, "y": fy, "yaw": fyaw},
        "scan": digest,
    }


# ---------------------------------------------------------------------------
# HTTP handlers
# ---------------------------------------------------------------------------

def make_handler(state: BridgeState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            # allow_nan=False forces ValueError on NaN/Inf rather than emitting
            # bare `NaN` literals that strict JSON parsers reject. Surface that
            # as a 500 with a clear hint so the agent can react.
            try:
                body = json.dumps(obj, allow_nan=False).encode("utf-8")
            except ValueError:
                body = json.dumps({
                    "error": "non-finite value in response (NaN or Inf)",
                    "hint": "pose_node may not be physical yet; wait a few ticks",
                }).encode("utf-8")
                code = 500
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _ok(self, extra=None):
            payload = {"status": "ok"}
            if extra:
                payload.update(extra)
            self._json(200, payload)

        def _read_json(self):
            n = int(self.headers.get("Content-Length") or "0")
            if n == 0:
                return {}
            raw = self.rfile.read(n).decode("utf-8") or "{}"
            return json.loads(raw)

        def do_GET(self):
            if self.path == "/state":
                self._json(200, state.snapshot())
                return
            if self.path == "/capabilities":
                self._json(200, {
                    "robot_id": "husky",
                    "model": "Clearpath Husky A200 (URDF)",
                    "wheel_motors": list(WHEEL_MOTORS),
                    "max_wheel_speed_radps": MAX_WHEEL_SPEED,
                    "wheel_radius_m": WHEEL_RADIUS_M,
                    "half_track_m": HALF_TRACK_M,
                    "max_linear_m_s": MAX_LINEAR_M_S,
                    "max_angular_r_s": MAX_ANGULAR_R_S,
                    "tick_period_s": state.tick_period_s,
                    "maze": MAZE,
                    "world_title": state.world_title,
                    "map_available": state.reveal_map,
                    "lidar_available": state.reveal_lidar,
                    "mission_brief": state.mission_brief,
                    "mission_complete": state.mission_complete,
                    "lidar": {
                        "num_rays": LIDAR_NUM_RAYS,
                        "max_range_m": LIDAR_MAX_RANGE_M,
                        "frame": "body",
                    },
                    # Only report camera_available=True when the husky_eye
                    # sidecar is up and serving 320x240 robot-frame images.
                    # The fallback operator-viewport path returns 800x600
                    # diagnostic frames (~3 MB base64) which crash the
                    # OmniLink chat at the 1 MB request limit; we don't
                    # want the agent thinking that's a usable sensor.
                    "camera_available": _probe_camera().get("kind") == "robot_camera",
                    "camera": _probe_camera(),
                })
                return
            if self.path == "/lidar":
                if not state.reveal_lidar:
                    self._json(200, {
                        "available": False,
                        "world_title": state.world_title,
                        "hint": (
                            "This world's title flags lidar as unavailable "
                            "('blind' world). Use /camera (read_camera) to "
                            "navigate; the bridge will not expose ray ranges."
                        ),
                    })
                    return
                snap = state.snapshot()
                if state.walls:
                    body_angles, ranges, hits = lidar_scan(
                        snap["x"], snap["y"], snap["yaw"], state.walls
                    )
                    # Only include hit names when the map is revealed —
                    # otherwise the agent could reverse-engineer the wall
                    # layout from per-ray hit identifiers.
                    payload = {
                        "angles_rad": body_angles,
                        "ranges_m": ranges,
                        "max_range_m": LIDAR_MAX_RANGE_M,
                        "pose": {"x": snap["x"], "y": snap["y"], "yaw": snap["yaw"]},
                    }
                    if state.reveal_map:
                        payload["hits"] = hits
                    self._json(200, payload)
                else:
                    self._json(503, {"error": "lidar walls not yet collected"})
                return
            if self.path == "/camera":
                snap = state.snapshot()
                # Preferred path: proxy to the husky_eye sidecar Robot's
                # /image endpoint on port 6071. The eye owns a real
                # Webots Camera device and tracks the husky's pose every
                # tick — see controllers/husky_eye/.
                try:
                    import urllib.request as _u
                    req = _u.Request(f"http://127.0.0.1:{EYE_SIDECAR_PORT}/image", method="GET")
                    with _u.urlopen(req, timeout=5) as r:
                        eye = json.loads(r.read().decode("utf-8", errors="replace"))
                    if "image_base64" in eye:
                        eye["pose"] = {"x": snap["x"], "y": snap["y"], "yaw": snap["yaw"]}
                        eye["view_kind"] = "robot_camera"
                        self._json(200, eye)
                        return
                except Exception:
                    # Eye sidecar not present / not ready. Refuse the
                    # camera read: the operator-viewport diagnostic
                    # fallback returns 800x600 frames (~3 MB base64)
                    # that have routinely tripped OmniLink's 1 MB
                    # request-size limit when packed into chat
                    # message history. Telling the agent the camera
                    # is unavailable is much cheaper than serving a
                    # frame the next chat() can't carry.
                    self._json(503, {
                        "available": False,
                        "world_title": state.world_title,
                        "hint": (
                            "This world has no husky_eye sidecar — "
                            "no real robot-frame camera is wired up. "
                            "capabilities.camera_available reports "
                            "false on this world; do not call "
                            "read_camera here. Use try_get_known_map "
                            "or read_lidar instead."
                        ),
                    })
                    return

                # Diagnostic-only path below was the old fallback. It is
                # no longer reachable because the except above returns
                # 503 unconditionally when the eye sidecar is missing.
                # Kept as code so the supervisor-exportImage plumbing
                # is still discoverable for anyone who wants to wire a
                # smaller diagnostic frame in future. NOT USED.
                # Defer the actual exportImage to the main thread via the
                # `vision_request` queue so we don't fight Webots' threading.
                with state.lock:
                    if state.vision_path is None:
                        self._json(503, {
                            "error": "vision capture not yet ready",
                            "hint": "Bridge is still warming up; retry in 1 s.",
                        })
                        return
                    state.vision_request += 1
                # Wait briefly for the main loop to refresh the file.
                req_id = state.vision_request
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    with state.lock:
                        served = state.vision_served
                    if served >= req_id:
                        break
                    time.sleep(0.05)
                try:
                    with open(state.vision_path, "rb") as f:
                        png_bytes = f.read()
                except Exception as exc:
                    self._json(500, {"error": f"could not read vision file: {exc}"})
                    return
                self._json(200, {
                    "width": state.vision_width,
                    "height": state.vision_height,
                    "encoding": "image/png; base64",
                    "image_base64": base64.b64encode(png_bytes).decode("ascii"),
                    "pose": {"x": snap["x"], "y": snap["y"], "yaw": snap["yaw"]},
                    "sim_time": snap["sim_time"],
                    "view_kind": "operator_viewport",
                })
                return
            if self.path == "/scan":
                # Local-perception proxy: forward to the husky_eye sidecar's
                # /scan, which numpy-analyses all cardinal frames and
                # returns a small structured dict (per-cardinal wall_close,
                # marker, marker_centroid, etc). Drops the agent's per-cell
                # input from ~120 K image tokens to ~1 K of JSON.
                if not _probe_camera().get("kind") == "robot_camera":
                    self._json(503, {
                        "available": False,
                        "world_title": state.world_title,
                        "hint": (
                            "scan_surroundings requires the husky_eye "
                            "sidecar (vision worlds only). "
                            "capabilities.camera_available reports false "
                            "on this world."
                        ),
                    })
                    return
                snap = state.snapshot()
                try:
                    import urllib.request as _u
                    req = _u.Request(f"http://127.0.0.1:{EYE_SIDECAR_PORT}/scan", method="GET")
                    with _u.urlopen(req, timeout=5) as r:
                        eye = json.loads(r.read().decode("utf-8", errors="replace"))
                    eye["pose"] = {"x": snap["x"], "y": snap["y"], "yaw": snap["yaw"]}
                    eye["current_cell"] = snap.get("current_cell")
                    eye["visited_cells"] = snap.get("visited_cells")
                    eye["unvisited_neighbours"] = snap.get("unvisited_neighbours")
                    self._json(200, eye)
                    return
                except Exception as exc:
                    self._json(502, {"error": f"sidecar /scan failed: {exc}"})
                    return
            if self.path.startswith("/solid"):
                # GET /solid?def=NAME — return the world position +
                # rotation of any DEF'd Solid in the scene. Used by
                # push_pallet_to to verify a pallet actually reached
                # its target after a shove (waypoint arrival on the
                # husky is not the same as the pallet getting where
                # it was supposed to go — it can collide with another
                # pallet, slide off the bumper, get stuck on a wall).
                # Also used by operators to ground-truth the world
                # state without screenshotting the GUI.
                from urllib.parse import urlparse, parse_qs as _parse_qs
                qs = _parse_qs(urlparse(self.path).query)
                def_name = (qs.get("def") or [""])[0].strip()
                if not def_name:
                    self._json(400, {
                        "error": "missing 'def' query param",
                        "example": "/solid?def=LOAD_GREEN",
                    })
                    return
                sup = getattr(state, "supervisor", None)
                if sup is None:
                    self._json(503, {"error": "supervisor not initialised"})
                    return
                node = sup.getFromDef(def_name)
                if node is None:
                    self._json(404, {
                        "error": f"no DEF named {def_name!r}",
                        "hint": "names are case-sensitive; the warehouse uses LOAD_RED, LOAD_GREEN, LOAD_BLUE, LOAD_YELLOW, LOAD_MAGENTA, LOAD_CYAN, DOCK, UR5E, LOADER_PEDESTAL.",
                    })
                    return
                try:
                    pos = list(node.getPosition())
                    orient = list(node.getOrientation())
                except Exception as exc:
                    self._json(500, {
                        "error": f"node {def_name!r} has no pose: {exc.__class__.__name__}: {exc}",
                    })
                    return
                self._json(200, {
                    "def": def_name,
                    "world_position": [float(p) for p in pos],
                    "world_orientation_3x3_row_major": [float(v) for v in orient],
                })
                return
            if self.path == "/mission":
                self._json(200, {
                    "world_title": state.world_title,
                    "brief": state.mission_brief,
                    "complete": state.mission_complete,
                    "log": list(state.mission_log),
                    "hint": (
                        "Read 'brief' in natural language. Plan + execute. "
                        "When you believe the mission is satisfied, POST "
                        "/action {action: 'complete_mission', rationale: "
                        "'<one-sentence why>', claimed_cells: [[col,row], "
                        "...] }. The bridge does not verify your claim — "
                        "it logs it. Operators audit the log."
                    ),
                })
                return
            if self.path == "/maze":
                if not state.reveal_map:
                    if state.reveal_lidar:
                        hint = (
                            "This world's title flags the map as unknown. "
                            "Use /lidar to navigate; the bridge will not "
                            "expose the wall list."
                        )
                    else:
                        hint = (
                            "This world's title flags both map AND lidar as "
                            "unavailable ('blind' world). Use /camera "
                            "(read_camera) and pose (get_state) to navigate; "
                            "neither the wall list nor ray ranges are exposed."
                        )
                    self._json(200, {
                        "available": False,
                        "world_title": state.world_title,
                        "hint": hint,
                    })
                    return
                adj = build_maze_graph_from_walls(state.walls)
                self._json(200, {
                    "available": True,
                    "world_title": state.world_title,
                    "cells": MAZE["cols"],
                    "cell_size_m": MAZE["cell_size_m"],
                    "origin_x": MAZE["origin_x"],
                    "origin_y": MAZE["origin_y"],
                    "start": MAZE["start"],
                    "goal": MAZE["goal"],
                    "adjacency": adj,
                })
                return
            self._json(404, {"error": "not found"})

        def do_POST(self):
            # /admin/reload — main-thread-deferred world reload. Optional
            # body {"world": "absolute/path.wbt"} switches worlds; no body
            # reloads the current world. Either way the controller process
            # is respawned, so the HTTP response may not arrive — clients
            # should treat a connection reset as success.
            if self.path == "/admin/reload":
                try:
                    body = self._read_json()
                except Exception:
                    body = {}
                with state.lock:
                    state.reload_request = body.get("world", "") or ""
                self._ok({"queued_reload": state.reload_request or "<current>"})
                return

            if self.path == "/admin/teleport_solid":
                # Teleport any DEF'd Solid via the supervisor. Used to
                # script "something moved" events for the warehouse_patrol
                # demo (operator runs a small Python script that pokes
                # this endpoint to relocate a crate between sweeps; the
                # next sweep's diff_sweeps then surfaces the change).
                # Body: {"def": "CRATE_GREEN", "x": 1.0, "y": 2.0, "z": 0.20}.
                try:
                    body = self._read_json()
                except Exception as exc:
                    self._json(400, {"error": f"bad json: {exc}"})
                    return
                def_name = str(body.get("def") or "").strip()
                if not def_name:
                    self._json(400, {"error": "'def' required",
                                     "example": "{\"def\":\"CRATE_GREEN\",\"x\":1,\"y\":2,\"z\":0.2}"})
                    return
                sup = getattr(state, "supervisor", None)
                if sup is None:
                    self._json(503, {"error": "supervisor not initialised"})
                    return
                node = sup.getFromDef(def_name)
                if node is None:
                    self._json(404, {"error": f"no DEF named {def_name!r}"})
                    return
                tf = node.getField("translation")
                if tf is None:
                    self._json(409, {"error": f"DEF {def_name!r} has no translation field"})
                    return
                try:
                    cur = list(tf.getSFVec3f())
                    nx = float(body.get("x", cur[0]))
                    ny = float(body.get("y", cur[1]))
                    nz = float(body.get("z", cur[2]))
                except Exception as exc:
                    self._json(400, {"error": f"bad coords: {exc}"})
                    return
                tf.setSFVec3f([nx, ny, nz])
                # Reset physics so the moved Solid doesn't carry stale
                # velocity from before the teleport (a 0.40 m crate that
                # had been resting on the floor before would briefly
                # rebound mid-air on a no-physics-reset move).
                try:
                    node.resetPhysics()
                except Exception:
                    pass
                self._ok({
                    "def": def_name,
                    "moved_from": [round(cur[0], 3), round(cur[1], 3), round(cur[2], 3)],
                    "moved_to": [round(nx, 3), round(ny, 3), round(nz, 3)],
                })
                return

            if self.path != "/action":
                self._json(404, {"error": "not found"})
                return
            try:
                body = self._read_json()
            except Exception as exc:
                self._json(400, {"error": f"bad json: {exc}"})
                return

            action = body.get("action", "")

            if action == "stop":
                with state.lock:
                    state.task = None
                    state.mode = "stopped"
                    state.fault = None
                self._ok({"halted_at": time.time()})
                return

            if action == "reset":
                with state.lock:
                    state.task = None
                    state.mode = "idle"
                    state.fault = None
                    state.reset_request = {
                        "x": MAZE["start"]["x"],
                        "y": MAZE["start"]["y"],
                        "yaw": 0.0,
                    }
                    # Mission state resets too — operator gets a fresh
                    # session every time they restart the husky.
                    state.mission_complete = False
                    state.mission_log = []
                    # Visited-cells episodic memory also clears on reset:
                    # a fresh session starts with no exploration record so
                    # the agent does not mistake stale breadcrumbs for
                    # current-mission progress.
                    state.visited_cells = set()
                self._ok({"reset_to": MAZE["start"]})
                return

            if action == "complete_mission":
                rationale = (body.get("rationale") or "").strip()
                claimed_cells_raw = body.get("claimed_cells") or []
                if not rationale:
                    self._json(400, {"error": "rationale is required (one-sentence why)"})
                    return

                # Normalize claimed cells into (col, row) tuples.
                claimed = []
                bad = []
                for c in claimed_cells_raw:
                    try:
                        claimed.append((int(c[0]), int(c[1])))
                    except (TypeError, ValueError, IndexError):
                        bad.append(c)
                if bad:
                    self._json(400, {
                        "error": "claimed_cells entries must be [col,row] integer pairs",
                        "rejected": bad,
                    })
                    return

                # Decide what counts as "actually completed" per world. The
                # bridge owns the rule because the brief is just free text;
                # only the bridge sees the ground-truth visited trail. Adding
                # a new world means adding a row here.
                with state.lock:
                    title_lower = (state.world_title or "").lower()
                    visited = set(tuple(c) for c in state.visited_cells)
                    gr = state.goal_reached
                # Default: legacy SE-goal worlds (basic / unknown / visual /
                # blind all stop at (10, 0)). Require goal_reached.
                required_cells = set()
                require_goal_reached = True
                if "corners" in title_lower:
                    # Multi-objective tour: visit every corner. goal_reached
                    # is meaningless here because there is no single goal.
                    required_cells = {(0, 0), (10, 0), (0, 10), (10, 10)}
                    require_goal_reached = False

                reasons = []
                bad_claims = [list(c) for c in claimed if c not in visited]
                if bad_claims:
                    reasons.append({
                        "kind": "claim_not_visited",
                        "missing_claimed_cells": bad_claims,
                        "hint": "every cell in claimed_cells must appear in the bridge's visited trail",
                    })
                missing_required = sorted(
                    [list(c) for c in required_cells if c not in visited]
                )
                if missing_required:
                    reasons.append({
                        "kind": "required_cells_not_visited",
                        "missing_required_cells": missing_required,
                        "world_title": state.world_title,
                    })
                if require_goal_reached and not gr:
                    reasons.append({
                        "kind": "goal_not_reached",
                        "hint": "husky must enter the goal cell tolerance for legacy SE-corner missions",
                    })

                ts = time.time()
                if reasons:
                    with state.lock:
                        state.mission_log.append({
                            "timestamp": ts,
                            "sim_time": state.sim_time,
                            "rationale": rationale,
                            "claimed_cells": [list(c) for c in claimed],
                            "verified": False,
                            "reasons": reasons,
                        })
                        last_entry = state.mission_log[-1]
                    self._json(409, {
                        "error": "mission claim refused: verification failed",
                        "verified": False,
                        "reasons": reasons,
                        "visited_cells_count": len(visited),
                        "log_entry": last_entry,
                    })
                    return

                with state.lock:
                    state.mission_complete = True
                    state.mission_log.append({
                        "timestamp": ts,
                        "sim_time": state.sim_time,
                        "rationale": rationale,
                        "claimed_cells": [list(c) for c in claimed],
                        "verified": True,
                    })
                    last_entry = state.mission_log[-1]
                self._ok({
                    "mission_complete": True,
                    "verified": True,
                    "logged_at": ts,
                    "log_entry": last_entry,
                })
                return

            if action == "snap_to_cell":
                # Teleport-based snapping has been removed globally. The
                # husky now navigates entirely under wheel control (see
                # goto_cell / drive_to_waypoint), so snap_to_cell no longer
                # teleports. Retained as an explicit no-op that reports the
                # cell centre it would have snapped to, so older callers get
                # a clean 410 rather than a silent behaviour change.
                self._json(410, {
                    "error": "snap_to_cell removed: teleport-snapping is disabled; "
                             "use goto_cell or drive_to_waypoint (wheel-driven).",
                })
                return

            if action == "set_velocity":
                try:
                    linear = float(body.get("linear", 0.0))
                    angular = float(body.get("angular", 0.0))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad velocity: {exc}"})
                    return
                linear = clamp(linear, -MAX_LINEAR_M_S, MAX_LINEAR_M_S)
                angular = clamp(angular, -MAX_ANGULAR_R_S, MAX_ANGULAR_R_S)
                with state.lock:
                    state.task = {
                        "kind": "velocity",
                        "linear": linear,
                        "angular": angular,
                    }
                    state.mode = "velocity"
                    state.fault = None
                self._ok({"linear": linear, "angular": angular})
                return

            if action == "drive_forward":
                try:
                    distance = float(body.get("distance", 0.0))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad distance: {exc}"})
                    return
                speed = clamp(
                    float(body.get("speed", 0.5)) * MAX_LINEAR_M_S,
                    0.05, MAX_LINEAR_M_S,
                )
                # Track signed progress along the *original* heading axis
                # rather than Euclidean distance to a fixed target. This
                # lets the controller (a) decelerate as it approaches the
                # commanded distance, (b) actively reverse if it overshoots,
                # and (c) hold heading the whole way. With a Euclidean
                # target, an overshoot makes the controller try to turn
                # around — fatal in a maze corridor.
                with state.lock:
                    sx, sy, syaw = state.x, state.y, state.yaw
                    hx = math.cos(syaw)
                    hy = math.sin(syaw)
                    state.task = {
                        "kind": "drive_forward",
                        "start_x": sx,
                        "start_y": sy,
                        "heading_x": hx,
                        "heading_y": hy,
                        "target_yaw": syaw,
                        "distance": distance,
                        "speed": speed,
                        "deadline": state.sim_time + GOTO_TIMEOUT_S,
                    }
                    state.mode = "drive_forward"
                    state.fault = None
                self._ok({"distance": distance, "speed_m_s": speed})
                return

            if action == "drive_to_waypoint":
                # Continuous-space "drive to (x, y)" — used by the Warehouse
                # Picker on warehouse_logistics.wbt where the husky navigates
                # arbitrary world coordinates between pallet stacks rather
                # than a fixed cell grid. Reuses the existing goto_cell
                # per-tick controller (two-phase turn-then-drive with a
                # heading-error gate) because that's exactly the right
                # control law for "go to a point"; the only difference
                # from goto_cell is that there's no cell-grid validation
                # or snap-to-grid post-step. Synchronous-wait mirrors
                # goto_cell's wait=true contract.
                try:
                    tx = float(body.get("x"))
                    ty = float(body.get("y"))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad x/y: {exc}"})
                    return
                speed = clamp(
                    float(body.get("speed", 0.5)) * MAX_LINEAR_M_S,
                    0.05, MAX_LINEAR_M_S,
                )
                # Default 90 s — GOTO_TIMEOUT_S (30 s) is sized for single
                # 2 m maze cell hops; warehouse waypoints can be 20+ m of
                # diagonal traversal at the husky's ~0.5 m/s average
                # (turn-then-drive loses momentum repeatedly). 90 s
                # comfortably covers the longest plausible warehouse hop.
                timeout_s = float(body.get("timeout_s", 90.0))
                with state.lock:
                    state.task = {
                        "kind": "goto_cell",  # reuse the existing controller
                        "col": None,
                        "row": None,
                        "x": tx,
                        "y": ty,
                        "speed": speed,
                        "deadline": state.sim_time + timeout_s,
                    }
                    state.mode = "drive_to_waypoint"
                    state.fault = None
                if body.get("wait"):
                    result = _wait_for_task_idle(state, timeout_s + 5.0)
                    fx, fy, fyaw = result["x"], result["y"], result["yaw"]
                    dist = math.hypot(fx - tx, fy - ty)
                    # Waypoint-mode tolerance is more permissive than the
                    # cell-grid GOTO_REACH_TOL_M (0.18 m): warehouses don't
                    # need cm accuracy, just "in the area". 0.60 m is the
                    # sweet spot for the warehouse_logistics layout —
                    # close enough to the pallet for the camera to read
                    # the tag (camera FoV at 1 m gives a 1.4 m visible
                    # patch on the pallet face), well within the
                    # 1.2 m pallet-to-pallet aisle width so the husky
                    # doesn't bump neighbours. Caller can pass
                    # `arrival_tolerance_m` for tighter or looser. If we
                    # got within tolerance, treat http_wait_timeout as
                    # success — the husky settled close enough.
                    arrival_tol = float(body.get("arrival_tolerance_m", 1.00))
                    # Promote "close enough" timeouts to success. Two
                    # timeout sources to handle: http_wait_timeout (the
                    # client-side wait gave up) and goto_cell_timeout
                    # (the controller's own deadline). Both mean "we
                    # didn't reach GOTO_REACH_TOL_M (0.18 m) in time" —
                    # but for warehouse waypoints, getting within
                    # arrival_tol of the target IS reaching it.
                    timeout_faults = {"http_wait_timeout", "goto_cell_timeout"}
                    if dist <= arrival_tol and result["fault"] in timeout_faults:
                        result_done = True
                        result_fault = None
                    else:
                        result_done = result["done"]
                        result_fault = result["fault"]
                    # Optional post-arrival aim. The goto_cell controller
                    # leaves the husky facing whichever direction the
                    # last-tick correction had it pointing — fine for
                    # the maze (cells are small + agent uses scan_surroundings
                    # to re-orient anyway), but on the warehouse the picker
                    # wants the camera pointed at the pallet it just drove
                    # look_at previously teleport-snapped the husky's yaw to
                    # face the target on arrival. Teleport-snapping has been
                    # removed globally, so the husky keeps the wheel-driven
                    # heading it arrived with. The ground-truth crate position
                    # (GET /solid) is what the patrol diff relies on, so this
                    # does not affect sweep accuracy; only the camera framing
                    # is no longer auto-aimed.
                    look = body.get("look_at")
                    looked_at = None
                    if (result_done and not result_fault
                            and isinstance(look, (list, tuple)) and len(look) == 2):
                        try:
                            lx = float(look[0]); ly = float(look[1])
                            looked_at = {"x": lx, "y": ly,
                                         "final_yaw": fyaw,
                                         "method": "none"}
                        except (TypeError, ValueError) as exc:
                            looked_at = {"error": f"bad look_at: {exc}"}
                    self._ok({
                        "x": tx, "y": ty,
                        "waited": True,
                        "done": result_done,
                        "fault": result_fault,
                        "final_pose": {"x": fx, "y": fy, "yaw": fyaw},
                        "distance_remaining_m": dist,
                        "arrival_tolerance_m": arrival_tol,
                        "looked_at": looked_at,
                    })
                    return
                self._ok({"x": tx, "y": ty, "speed_m_s": speed})
                return

            if action == "turn":
                try:
                    angle = float(body.get("angle", 0.0))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad angle: {exc}"})
                    return
                speed = clamp(
                    float(body.get("speed", 0.6)) * MAX_ANGULAR_R_S,
                    0.0, MAX_ANGULAR_R_S,
                )
                with state.lock:
                    state.task = {
                        "kind": "turn",
                        "target_yaw": wrap_pi(state.yaw + angle),
                        "speed": speed,
                        "deadline": state.sim_time + GOTO_TIMEOUT_S,
                    }
                    state.mode = "turn"
                    state.fault = None
                self._ok({"angle_rad": angle, "speed_rad_s": speed})
                return

            if action == "goto_cell":
                try:
                    col = int(body.get("col"))
                    row = int(body.get("row"))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad cell: {exc}"})
                    return
                if not (0 <= col < MAZE["cols"] and 0 <= row < MAZE["rows"]):
                    self._json(400, {
                        "error": "cell out of range",
                        "valid_cols": [0, MAZE["cols"] - 1],
                        "valid_rows": [0, MAZE["rows"] - 1],
                    })
                    return
                # Adjacency gate. Source cell is the husky's ACTUAL rounded
                # pose, not the agent-supplied prev_cell — otherwise the
                # agent can lie about where the husky is, the bridge accepts
                # a "legal" hop, the wheels fail to cover the actual distance,
                # and the wedge-recovery snap teleports the husky across the
                # maze. prev_cell is still consulted for snap yaw inference
                # below, but legality runs off ground truth.
                with state.lock:
                    walls_snap = list(state.walls)
                    actual_src = (
                        int(round((state.x - MAZE["origin_x"]) / MAZE["cell_size_m"])),
                        int(round((state.y - MAZE["origin_y"]) / MAZE["cell_size_m"])),
                    )
                if not is_legal_hop(walls_snap, actual_src, (col, row)):
                    self._json(400, {
                        "error": "non_adjacent_hop",
                        "current_cell": list(actual_src),
                        "target": [col, row],
                        "hint": (
                            "goto_cell only drives to a 4-cardinal neighbour "
                            "of the husky's CURRENT cell with no wall between. "
                            "Call get_state to see current_cell, then plan_path "
                            "for the legal hop sequence; do NOT pass prev_cell "
                            "to override this check."
                        ),
                    })
                    return
                speed = clamp(
                    float(body.get("speed", 0.5)) * MAX_LINEAR_M_S,
                    0.05, MAX_LINEAR_M_S,
                )
                tx = MAZE["origin_x"] + col * MAZE["cell_size_m"]
                ty = MAZE["origin_y"] + row * MAZE["cell_size_m"]
                # (Teleport pre-snap removed globally. goto_cell already turns
                # in place under wheel control before driving, so no re-anchor
                # is needed; the husky moves entirely on its wheels.)
                with state.lock:
                    state.task = {
                        "kind": "goto_cell",
                        "col": col,
                        "row": row,
                        "x": tx,
                        "y": ty,
                        "speed": speed,
                        "deadline": state.sim_time + GOTO_TIMEOUT_S,
                    }
                    state.mode = "goto_cell"
                    state.fault = None
                # Optional synchronous wait. Default is the legacy fire-and-forget
                # behavior so v1 callers still see the same response shape. v2
                # passes wait=true and gets one response per move instead of having
                # to poll /state until mode flips to idle. With auto_snap_threshold_m
                # set the bridge also re-anchors to the cell centre when residual
                # drift exceeds the threshold (cardinal yaw inferred from prev_cell
                # if supplied, else preserved).
                if body.get("wait"):
                    timeout_s = float(body.get("timeout_s", GOTO_TIMEOUT_S + 5.0))
                    result = _wait_for_task_idle(state, timeout_s)
                    drift = math.hypot(result["x"] - tx, result["y"] - ty)
                    fx, fy, fyaw = result["x"], result["y"], result["yaw"]
                    arrival_tol = float(
                        body.get("arrival_tolerance_m", MAZE["cell_size_m"] / 2.0)
                    )
                    result_fault = result["fault"]
                    result_done = result["done"]
                    # Close-enough acceptance (no teleport — purely relaxes
                    # the success tolerance). The wheel-driven controller
                    # often overruns its sim-time deadline on 90° corner
                    # hops where the in-place pivot eats most of the budget,
                    # but ends up inside arrival_tol. Don't fault — the next
                    # hop's controller starts from the actual settled pose.
                    if (drift <= arrival_tol
                            and result_fault in ("goto_cell_timeout", "http_wait_timeout")):
                        result_fault = None
                        result_done = True
                    self._ok({
                        "col": col, "row": row, "x": tx, "y": ty,
                        "waited": True,
                        "done": result_done,
                        "fault": result_fault,
                        "final_pose": {"x": fx, "y": fy, "yaw": fyaw},
                        "drift_m": drift,
                        "arrival_tolerance_m": arrival_tol,
                    })
                    return
                self._ok({"col": col, "row": row, "x": tx, "y": ty})
                return

            if action == "execute_path":
                # Drive a list of cells in sequence. Bridge handles per-cell
                # pose-settle waits and an optional drift-gated snap so the
                # agent can plan once and call once. Used by v3.
                cells = body.get("cells")
                if not isinstance(cells, list) or not cells:
                    self._json(400, {"error": "cells: non-empty list of [col,row] required"})
                    return
                # Adjacency gate. Walk the whole chain before driving anything:
                # first cell against the husky's ACTUAL current cell (not any
                # agent-supplied start), every subsequent cell against its
                # predecessor. Refuse on the first illegal hop so the
                # wedge-recovery snap never gets to teleport over a wall (or
                # across the maze when the agent picks up a stale start).
                with state.lock:
                    walls_snap = list(state.walls)
                    actual_src = (
                        int(round((state.x - MAZE["origin_x"]) / MAZE["cell_size_m"])),
                        int(round((state.y - MAZE["origin_y"]) / MAZE["cell_size_m"])),
                    )
                blocked = build_blocked_edges(walls_snap)
                cells_norm = []
                prev_cr = actual_src
                for idx, c in enumerate(cells):
                    try:
                        cc, cr = int(c[0]), int(c[1])
                    except (TypeError, ValueError, IndexError):
                        self._json(400, {
                            "error": f"bad_cell_at_index_{idx}",
                            "cell": c,
                        })
                        return
                    if not (0 <= cc < MAZE["cols"] and 0 <= cr < MAZE["rows"]):
                        self._json(400, {
                            "error": f"cell_out_of_range_at_{idx}",
                            "cell": [cc, cr],
                        })
                        return
                    target = (cc, cr)
                    if target != prev_cr:
                        if (abs(prev_cr[0] - cc) + abs(prev_cr[1] - cr) != 1
                                or (prev_cr, target) in blocked):
                            self._json(400, {
                                "error": "non_adjacent_hop",
                                "first_illegal_hop": {
                                    "index": idx,
                                    "from": list(prev_cr),
                                    "to": [cc, cr],
                                },
                                "cells_validated": idx,
                                "hint": (
                                    "Every consecutive pair in `cells` must be "
                                    "4-cardinal neighbours with no wall between. "
                                    "BFS over /maze adjacency before calling "
                                    "execute_path; do NOT rely on the wedge "
                                    "snap to clip walls."
                                ),
                            })
                            return
                    cells_norm.append(target)
                    prev_cr = target
                speed = clamp(
                    float(body.get("speed", 0.5)) * MAX_LINEAR_M_S,
                    0.05, MAX_LINEAR_M_S,
                )
                # snap_drift_threshold_m is accepted for back-compat with
                # callers from before c50e9318 but is now a no-op (no teleport
                # snapping in any form).
                snap_thresh = float(body.get("snap_drift_threshold_m", 0.40))
                per_cell_timeout = float(body.get("per_cell_timeout_s", GOTO_TIMEOUT_S))
                cells_done = []
                fault = None
                fx = fy = fyaw = None
                start_sim = None
                with state.lock:
                    start_sim = state.sim_time
                for idx, cell in enumerate(cells):
                    try:
                        col = int(cell[0]); row = int(cell[1])
                    except Exception:
                        fault = f"bad_cell_at_index_{idx}"
                        break
                    if not (0 <= col < MAZE["cols"] and 0 <= row < MAZE["rows"]):
                        fault = f"cell_out_of_range_at_{idx}"
                        break
                    tx = MAZE["origin_x"] + col * MAZE["cell_size_m"]
                    ty = MAZE["origin_y"] + row * MAZE["cell_size_m"]
                    # (Teleport pre-snap removed globally. goto_cell turns in
                    # place under wheel control before driving, so each hop
                    # steers itself without a re-anchor.)
                    with state.lock:
                        state.task = {
                            "kind": "goto_cell",
                            "col": col,
                            "row": row,
                            "x": tx,
                            "y": ty,
                            "speed": speed,
                            "deadline": state.sim_time + per_cell_timeout,
                        }
                        state.mode = "goto_cell"
                        state.fault = None
                    res = _wait_for_task_idle(state, per_cell_timeout + 5.0)
                    fx, fy, fyaw = res["x"], res["y"], res["yaw"]
                    cell_fault = res["fault"]
                    drift = math.hypot(fx - tx, fy - ty)
                    arrival_tol = float(body.get(
                        "arrival_tolerance_m", MAZE["cell_size_m"] / 2.0
                    ))
                    # Close-enough acceptance (no teleport — wheel-only).
                    if cell_fault == "goto_cell_timeout" and drift <= arrival_tol:
                        cell_fault = None
                    if cell_fault:
                        cells_done.append({
                            "col": col, "row": row, "reached": False,
                            "drift_m": drift, "fault": cell_fault,
                        })
                        fault = cell_fault
                        break
                    _ = snap_thresh  # accepted for back-compat, no longer used
                    cells_done.append({
                        "col": col, "row": row, "reached": True,
                        "drift_m": drift, "fault": None,
                    })
                with state.lock:
                    end_sim = state.sim_time
                self._ok({
                    "cells_total": len(cells),
                    "cells_executed": len(cells_done),
                    "fault": fault,
                    "final_pose": ({"x": fx, "y": fy, "yaw": fyaw} if fx is not None else None),
                    "sim_seconds": (end_sim - start_sim) if start_sim is not None else None,
                    "cells": cells_done,
                })
                return

            if action == "walk_one_cell":
                # One world-cardinal hop with internal pre-snap, drive, post-snap,
                # and a wall_close safety refusal. Replaces the v1 cell-step
                # pattern of {scan + turn + drive + snap + scan} (4-5 chat turns)
                # with one tool call (1 chat turn). The agent passes only
                # `cardinal` ∈ {north, south, east, west}; the bridge handles
                # the body-frame mapping. Returns a digest scan from the new
                # cell, so the next decision needs no separate scan call.
                cardinal = body.get("cardinal", "")
                speed_frac = float(body.get("speed", 0.5))
                result = _walk_one_cell_inline(state, cardinal, speed_frac)
                self._json(200 if result.get("ok") else 200, result)
                return

            if action == "follow_corridor":
                # Walk in a world cardinal until: (a) a wall blocks the way,
                # (b) a coloured marker becomes prominent (when stop_on_marker),
                # or (c) max_cells reached. One chat turn drives an arbitrary
                # number of cells through a straight corridor. The classic
                # 4-cells-east case collapses from 12+ chat turns to 1.
                cardinal = body.get("cardinal", "")
                if cardinal not in CARDINAL_TO_YAW:
                    self._json(400, {"error": f"cardinal must be one of {list(CARDINAL_TO_YAW)}"})
                    return
                try:
                    max_cells = int(body.get("max_cells", 8))
                except (TypeError, ValueError):
                    self._json(400, {"error": "max_cells must be int"})
                    return
                max_cells = max(1, min(max_cells, MAZE["cols"] * MAZE["rows"]))
                speed_frac = float(body.get("speed", 0.5))
                stop_on_marker = bool(body.get("stop_on_marker", True))
                cells_walked = []
                stop_reason = None
                last_result = None
                # Iter 0 needs the safety scan (we haven't verified this
                # cardinal yet). Iter 1+ trusts the previous step's
                # post-walk digest: if it said `cardinal` is open, we
                # know the next walk is safe and can skip the racy
                # back-to-back /scan call.
                trust_prev_digest = False
                for step_idx in range(max_cells):
                    last_result = _walk_one_cell_inline(
                        state, cardinal, speed_frac,
                        safety_scan=(not trust_prev_digest),
                    )
                    if not last_result.get("ok"):
                        # First-step failure (e.g. wall_close immediately) →
                        # surface the refusal so the agent picks a different
                        # cardinal. Mid-run failure → stop with what we got.
                        stop_reason = last_result.get("fault") or "step_failed"
                        if step_idx == 0:
                            stop_reason = last_result.get("fault") or "first_step_blocked"
                        break
                    cells_walked.append(last_result.get("to_cell"))
                    digest = last_result.get("scan") or {}
                    marker = digest.get("marker")
                    if stop_on_marker and marker and (
                            marker.get("approach_recommended") or marker.get("adjacent")):
                        stop_reason = "marker_seen"
                        break
                    # Look-ahead: if next step in same cardinal is now blocked,
                    # stop the loop here so the agent can decide next cardinal.
                    if cardinal in (digest.get("blocked") or []):
                        stop_reason = "next_blocked"
                        break
                    # The post-walk digest just said `cardinal` is open at the
                    # new cell — the next iteration can skip its safety scan.
                    trust_prev_digest = (cardinal in (digest.get("open") or []))
                if stop_reason is None:
                    stop_reason = "max_cells"
                self._json(200, {
                    "ok": bool(cells_walked) or last_result is None,
                    "cardinal": cardinal,
                    "cells_walked": cells_walked,
                    "cells_total": len(cells_walked),
                    "stop_reason": stop_reason,
                    "scan": (last_result or {}).get("scan"),
                    "final_pose": (last_result or {}).get("final_pose"),
                    "first_refusal": (last_result if (last_result and not last_result.get("ok")) else None),
                })
                return

            if action == "auto_explore":
                # DFS frontier-explorer with optional marker chase. Runs the
                # whole "scan → pick cardinal → walk → repeat" loop server-
                # side so the LLM only intervenes once: it kicks off the
                # explore and reads the result. Replaces the per-cell chat
                # round-trip on the blind world with one tool call for the
                # entire mission.
                #
                # Strategy:
                #   1. Each tick, look at the perception digest at the current
                #      cell.
                #   2. If `target_color` is set and the digest reports a
                #      matching marker:
                #        - adjacent (wall_close + marker_fraction > 0.20):
                #          marker is in the next cell. If step_into_marker,
                #          walk that cardinal one more time to enter the
                #          marker cell, then stop. Else stop here.
                #        - approach_recommended (marker visible, not
                #          wall_close): walk toward it.
                #   3. Else pick a cardinal: prefer `unvisited_open`. If
                #      none, backtrack along the parent chain to the most
                #      recent cell with unvisited neighbours. If the chain
                #      is exhausted, stop with no_unvisited.
                #
                # The agent's contribution shrinks to: read brief, kick off
                # auto_explore("red"), inspect the result, complete_mission.
                target_color = body.get("target_color")
                if target_color is not None and target_color not in ("red", "green", "blue"):
                    self._json(400, {"error": "target_color must be one of red/green/blue or null"})
                    return
                try:
                    max_cells = int(body.get("max_cells", 50))
                except (TypeError, ValueError):
                    self._json(400, {"error": "max_cells must be int"})
                    return
                max_cells = max(1, min(max_cells, MAZE["cols"] * MAZE["rows"] * 4))
                speed_frac = float(body.get("speed", 0.5))
                step_into_marker = bool(body.get("step_into_marker", True))

                # Initial scan to seed the loop.
                with state.lock:
                    cur_x, cur_y, cur_yaw = state.x, state.y, state.yaw
                    visited_now = list(state.visited_cells)
                cur_col = int(round((cur_x - MAZE["origin_x"]) / MAZE["cell_size_m"]))
                cur_row = int(round((cur_y - MAZE["origin_y"]) / MAZE["cell_size_m"]))
                eye = _fetch_eye_scan()
                if not eye:
                    self._json(503, {"error": "eye sidecar unavailable"})
                    return
                digest = _make_scan_digest(eye, cur_yaw, [cur_col, cur_row], visited_now)

                # Parent map for backtracking: parent[(c,r)] = (c',r') or
                # None for the start cell.
                parent = {(cur_col, cur_row): None}
                cells_walked = []
                attempted_dead_end = set()  # (cell, cardinal) we've already failed
                stop_reason = None
                marker_found = None
                last_result = None

                # Pre-loop check: maybe the brief is satisfied at the
                # start cell already (marker visible adjacent).
                if target_color and digest.get("marker"):
                    m = digest["marker"]
                    if m.get("color") == target_color and m.get("adjacent") and not step_into_marker:
                        stop_reason = "marker_adjacent_at_start"
                        marker_found = m

                for step_idx in range(max_cells):
                    if stop_reason:
                        break

                    # Decide next cardinal.
                    next_cardinal = None
                    chosen_reason = ""

                    # 1. If a target marker is visible, chase it.
                    if target_color and digest.get("marker"):
                        m = digest["marker"]
                        if m.get("color") == target_color:
                            if m.get("adjacent"):
                                if step_into_marker:
                                    next_cardinal = m["world_cardinal"]
                                    chosen_reason = "step_into_marker"
                                else:
                                    stop_reason = "marker_adjacent"
                                    marker_found = m
                                    break
                            elif m.get("approach_recommended"):
                                next_cardinal = m["world_cardinal"]
                                chosen_reason = "approach_marker"

                    # 2. Otherwise prefer an unvisited open cardinal.
                    if not next_cardinal:
                        unvisited_open = [
                            c for c in (digest.get("unvisited_open") or [])
                            if ((cur_col, cur_row), c) not in attempted_dead_end
                        ]
                        if unvisited_open:
                            next_cardinal = unvisited_open[0]
                            chosen_reason = "unvisited_open"

                    # 3. Backtrack: walk toward the parent of the current
                    # cell. If we are at the start cell with nothing
                    # unvisited, the maze is exhausted.
                    if not next_cardinal:
                        p = parent.get((cur_col, cur_row))
                        if p is None:
                            stop_reason = "exhausted_no_unvisited"
                            break
                        dc = p[0] - cur_col
                        dr = p[1] - cur_row
                        if dc > 0:    next_cardinal = "east"
                        elif dc < 0:  next_cardinal = "west"
                        elif dr > 0:  next_cardinal = "north"
                        elif dr < 0:  next_cardinal = "south"
                        else:
                            stop_reason = "backtrack_self_loop"
                            break
                        chosen_reason = "backtrack"

                    # Walk one cell. safety_scan is False because we just
                    # consumed a fresh digest above; the per-call safety
                    # scan in walk_one_cell is racy and unnecessary here.
                    last_result = _walk_one_cell_inline(
                        state, next_cardinal, speed_frac, safety_scan=False,
                    )
                    if not last_result.get("ok"):
                        # The chosen cardinal turned out to be a wall the
                        # perception missed (drive_forward_timeout) or
                        # off-grid. Mark it as a dead end at this cell so
                        # we don't re-pick it, and try again with a fresh
                        # digest.
                        attempted_dead_end.add(((cur_col, cur_row), next_cardinal))
                        eye = _fetch_eye_scan()
                        if eye:
                            with state.lock:
                                cur_yaw = state.yaw
                                visited_now = list(state.visited_cells)
                            digest = _make_scan_digest(eye, cur_yaw, [cur_col, cur_row], visited_now)
                        continue

                    # Successful walk. Record parent + advance.
                    new_cell_list = last_result.get("to_cell") or [cur_col, cur_row]
                    new_cell = (int(new_cell_list[0]), int(new_cell_list[1]))
                    cells_walked.append(list(new_cell))
                    if new_cell not in parent:
                        parent[new_cell] = (cur_col, cur_row)
                    cur_col, cur_row = new_cell
                    digest = last_result.get("scan") or {}

                    # Post-walk marker check: if we just stepped INTO a
                    # marker cell as the final reach action, we're done.
                    if (target_color and chosen_reason == "step_into_marker"
                            and digest.get("marker")
                            and digest["marker"].get("color") != target_color):
                        # We drove past the marker (no longer visible from
                        # the new cell) — that means we ARE in its cell.
                        stop_reason = "marker_reached"
                        marker_found = {"color": target_color, "cell": [cur_col, cur_row]}
                        break
                    # If we step_into_marker and the marker is STILL
                    # visible from the new cell, we may have actually
                    # entered the marker cell (the camera now sees through
                    # / past it). Treat as reached.
                    if chosen_reason == "step_into_marker":
                        stop_reason = "marker_reached"
                        marker_found = {"color": target_color, "cell": [cur_col, cur_row]}
                        break

                if stop_reason is None:
                    stop_reason = "max_cells"

                self._json(200, {
                    "ok": stop_reason in ("marker_reached", "marker_adjacent",
                                          "marker_adjacent_at_start"),
                    "stop_reason": stop_reason,
                    "marker_found": marker_found,
                    "cells_walked": cells_walked,
                    "cells_total": len(cells_walked),
                    "final_cell": [cur_col, cur_row],
                    "scan": digest if isinstance(digest, dict) else None,
                    "final_pose": (last_result or {}).get("final_pose"),
                    "dead_ends_hit": len(attempted_dead_end),
                })
                return

            self._json(400, {"error": f"unknown action: {action}"})

    return Handler


def start_http(state: BridgeState):
    server = ThreadingHTTPServer((BRIDGE_HOST, BRIDGE_PORT), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[husky_omnilink_bridge] HTTP listening on http://{BRIDGE_HOST}:{BRIDGE_PORT}")
    return server


# ---------------------------------------------------------------------------
# Per-tick controllers
# ---------------------------------------------------------------------------

def _heading_to_point(x, y, yaw, tx, ty):
    dx, dy = tx - x, ty - y
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0, 0.0
    desired = math.atan2(dy, dx)
    return wrap_pi(desired - yaw), math.hypot(dx, dy)


SETTLE_LIN_M_S = 0.05    # consider the body stopped below this linear speed
SETTLE_ANG_R_S = 0.10    # and this angular speed


def _twist_for_task(task, x, y, yaw, sim_time, v_lin, v_ang):
    """Return (linear_m_s, angular_r_s, done, fault).

    `done` triggers task clear; `fault` (if non-empty) sets state.fault.
    `v_lin` and `v_ang` are the measured body speeds — used to gate task
    completion until the husky has actually come to a stop, so commands
    don't accumulate overshoot across cells."""
    kind = task["kind"]

    if kind == "velocity":
        return task["linear"], task["angular"], False, None

    if kind == "drive_forward":
        if sim_time > task["deadline"]:
            return 0.0, 0.0, True, "drive_forward_timeout"
        # Signed progress along the original heading axis.
        progress = (
            (x - task["start_x"]) * task["heading_x"]
            + (y - task["start_y"]) * task["heading_y"]
        )
        remaining = task["distance"] - progress  # positive = still ahead
        # Done when within tolerance of the commanded distance AND stopped.
        if abs(remaining) < GOTO_REACH_TOL_M:
            if abs(v_lin) < SETTLE_LIN_M_S and abs(v_ang) < SETTLE_ANG_R_S:
                return 0.0, 0.0, True, None
            return 0.0, 0.0, False, None
        # Direction of effort: forward if we still need progress, reverse
        # if we overshot. Magnitude tapers off near the end so we can stop
        # without coasting past.
        decel_factor = min(1.0, max(0.15, abs(remaining) / 0.7))
        lin = math.copysign(task["speed"] * decel_factor, remaining)
        # Heading correction back to the original axis. Use a softer gain
        # than turn-in-place so we don't fishtail.
        err_yaw = wrap_pi(task["target_yaw"] - yaw)
        if remaining < 0:
            # Reversing — desired heading axis is the original, but we're
            # driving backward, so the body should still face forward.
            pass
        ang = clamp(err_yaw * HEADING_KP * 0.6, -MAX_ANGULAR_R_S * 0.5, MAX_ANGULAR_R_S * 0.5)
        return lin, ang, False, None

    if kind == "turn":
        if sim_time > task["deadline"]:
            return 0.0, 0.0, True, "turn_timeout"
        err = wrap_pi(task["target_yaw"] - yaw)
        if abs(err) < ALIGN_TOL_RAD:
            # Same settle gate: don't release until rotation has stopped.
            if abs(v_ang) < SETTLE_ANG_R_S:
                return 0.0, 0.0, True, None
            return 0.0, 0.0, False, None
        ang = clamp(err * HEADING_KP, -task["speed"], task["speed"])
        return 0.0, ang, False, None

    if kind == "goto_cell":
        if sim_time > task["deadline"]:
            return 0.0, 0.0, True, "goto_cell_timeout"
        err_yaw, dist = _heading_to_point(x, y, yaw, task["x"], task["y"])
        if dist < GOTO_REACH_TOL_M:
            if abs(v_lin) < SETTLE_LIN_M_S and abs(v_ang) < SETTLE_ANG_R_S:
                return 0.0, 0.0, True, None
            return 0.0, 0.0, False, None
        # Two-phase: turn in place while the heading error is large, then
        # drive forward with a small heading correction. This is far more
        # robust on a 2 m maze grid than smooth pure-pursuit, which loves
        # to overshoot 90-degree turns and wedge against the next wall.
        # Pivot is capped at PIVOT_ANGULAR_R_S_MAX (1.5 rad/s) so the
        # skid-steer doesn't slide the body during in-place rotation —
        # critical for legal-only navigation (no teleport recovery).
        if abs(err_yaw) > TURN_FIRST_RAD:
            ang = clamp(err_yaw * HEADING_KP, -PIVOT_ANGULAR_R_S_MAX, PIVOT_ANGULAR_R_S_MAX)
            return 0.0, ang, False, None
        ang = clamp(err_yaw * HEADING_KP, -MAX_ANGULAR_R_S * 0.5, MAX_ANGULAR_R_S * 0.5)
        # Decelerate as we close on the target so we don't overshoot into the
        # next wall. With wheel ramping at 0.15 rad/s per 16 ms tick and a
        # ~1 m/s top speed, a 0.7 m decel window gives the body time to stop
        # before crossing the cell centre.
        distance_factor = min(1.0, max(0.15, dist / 0.7))
        align = max(0.2, math.cos(err_yaw))
        lin = task["speed"] * align * distance_factor
        return lin, ang, False, None

    return 0.0, 0.0, True, f"unknown_task:{kind}"


# ---------------------------------------------------------------------------
# Main supervisor loop
# ---------------------------------------------------------------------------

def main():
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())

    self_node = robot.getSelf()
    if self_node is None:
        print("[husky_omnilink_bridge] ERROR: getSelf() returned None - "
              "is supervisor TRUE on the Husky?")
        return

    motors = []
    for name in WHEEL_MOTORS:
        m = robot.getDevice(name)
        if m is None:
            print(f"[husky_omnilink_bridge] ERROR: motor {name!r} not found")
            return
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors.append(m)
    left_motors = motors[:2]
    right_motors = motors[2:]

    # Vision via a runtime-injected Webots Camera device. We can't put
    # a Camera in the URDFRobot's explicit children field (it blocks the
    # URDF expansion — wheels disappear) and the Webots URDF importer
    # doesn't parse <sensor type="camera"> tags. So instead we let the
    # URDF expand normally, then use the supervisor to inject a Camera
    # node into base_link.children at runtime. Webots auto-registers
    # the new device, and getDevice("front_camera") finds it after the
    # next step.
    camera = None
    cam_w = cam_h = 0
    cam_fov = 0.0

    # Translation/rotation fields drive the reset action. They live on
    # the URDFRobot wrapper node, not on the inner base_link.
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")

    # Prime URDF expansion before walking the subtree (mirrors
    # omnilink_arm_bridge). Two steps is what that controller uses.
    robot.step(time_step)
    robot.step(time_step)

    # The URDFRobot wrapper has no physics body — its getPosition()
    # returns NaN. Find a real child Solid (base_link is preferred; any
    # physical descendant works as a fallback) and read pose off it.
    pose_node = (
        find_link_by_name(self_node, "base_link")
        or find_link_by_name(self_node, "base_footprint")
        or find_first_physical_child(self_node)
    )
    if pose_node is None:
        print("[husky_omnilink_bridge] ERROR: could not find any child Solid "
              "with a finite world pose under the URDFRobot.")
        return
    if pose_node is self_node:
        print("[husky_omnilink_bridge] WARNING: pose_node is the URDFRobot itself; "
              "pose may be NaN if its body has no physics.")
    else:
        try:
            pn_name = pose_node.getField("name").getSFString()
        except Exception:
            pn_name = "<unnamed>"
        try:
            pn_type = pose_node.getTypeName()
        except Exception:
            pn_type = "?"
        try:
            pn_pos = pose_node.getPosition()
            pn_pos_str = f"({pn_pos[0]:.2f}, {pn_pos[1]:.2f}, {pn_pos[2]:.2f})"
        except Exception:
            pn_pos_str = "?"
        print(f"[husky_omnilink_bridge] pose_node = {pn_name!r} type={pn_type} "
              f"world_pos={pn_pos_str}")

    # NOTE: We don't try to mount a Camera on the husky URDFRobot here.
    # Three things have been verified not to work:
    #   1. Camera in URDFRobot.children block — kills URDF expansion.
    #   2. <sensor type="camera"> in the URDF — Webots' URDF importer
    #      doesn't parse Gazebo sensor extensions.
    #   3. importMFNodeFromString into base_link.children at runtime —
    #      Webots doesn't register dynamically-added devices.
    # Vision is provided by a SEPARATE Robot named "husky_eye" with its
    # own controller `husky_eye`. The eye Robot owns the Camera, tracks
    # the husky's pose each tick, and serves images over HTTP on port
    # 6071. Our /camera endpoint proxies to it (see _proxy_eye() below).

    state = BridgeState()
    state.tick_period_s = time_step / 1000.0
    # Stash the supervisor so the /solid endpoint can read DEF'd Solid
    # positions on demand. Used by push_pallet_to to verify a pallet
    # actually reached its target rather than trust waypoint arrival.
    state.supervisor = robot
    if camera is not None:
        state.camera = camera
        state.camera_width = cam_w
        state.camera_height = cam_h
        state.camera_fov_rad = cam_fov
    # Pick a writable temp file for vision exports. Use the controller
    # directory so we don't depend on /tmp existing on Windows.
    import os as _os
    import tempfile
    state.vision_path = _os.path.join(
        tempfile.gettempdir(), f"husky_bridge_vision_{_os.getpid()}.png"
    )

    # Walls + world title for /lidar and /maze. Done once at startup —
    # the maze is static, so caching is safe and the lidar handler can
    # read state.walls from any thread.
    state.walls = collect_walls(robot)
    title = ""
    info_lines: list = []
    try:
        world_info = robot.getFromDef("WORLDINFO")  # rarely set; usually None
        if world_info is None:
            root_children = robot.getRoot().getField("children")
            for i in range(root_children.getCount()):
                node = root_children.getMFNode(i)
                if node and node.getTypeName() == "WorldInfo":
                    world_info = node
                    break
        if world_info is not None:
            title_field = world_info.getField("title")
            if title_field is not None:
                title = title_field.getSFString() or ""
            # info is MFString — multi-line free-form text. We treat the
            # joined lines as the mission brief.
            info_field = world_info.getField("info")
            if info_field is not None:
                try:
                    n = info_field.getCount()
                    for i in range(n):
                        info_lines.append(info_field.getMFString(i))
                except Exception:
                    pass
    except Exception as exc:
        print(f"[husky_omnilink_bridge] could not read WorldInfo title: {exc}")
    state.world_title = title
    state.reveal_map = "unknown" not in title.lower() and "blind" not in title.lower()
    state.reveal_lidar = "blind" not in title.lower()
    state.mission_brief = "\n".join(info_lines).strip()
    if not state.mission_brief:
        state.mission_brief = (
            "(no WorldInfo.info brief set — fall back to the goal_marker "
            "Solid in the SE corner if present)"
        )
    print(f"[husky_omnilink_bridge] world_title = {title!r} "
          f"reveal_map = {state.reveal_map} walls = {len(state.walls)}")
    print(f"[husky_omnilink_bridge] mission_brief = {state.mission_brief[:120]!r}")

    # Prime pose so /state isn't zeroed before the first physics tick.
    pos0 = pose_node.getPosition()
    yaw0 = yaw_from_orientation(pose_node.getOrientation())
    with state.lock:
        state.x = float(pos0[0])
        state.y = float(pos0[1])
        state.yaw = yaw0
        state.last_tick_at = time.time()

    start_http(state)
    print(f"[husky_omnilink_bridge] ready. start={MAZE['start']} "
          f"goal={MAZE['goal']}")

    prev_x, prev_y, prev_yaw = state.x, state.y, state.yaw
    cur_left = cur_right = 0.0    # ramped wheel commands, in rad/s
    goal_x, goal_y = MAZE["goal"]["x"], MAZE["goal"]["y"]
    goal_radius = MAZE["goal_radius_m"]
    # Wedge-escape: when the controller commands motion but the body
    # is physically stuck against a wall, drive backward while pivoting
    # opposite to the last commanded pivot. The combined reverse + unwind
    # clears the most common skid-into-wall wedges. Pure wheel motion —
    # no teleport, no snap. The agent's drive_to_cell handles cases the
    # bridge controller can't crack by replanning from the new pose, so
    # the bridge doesn't need to be heroic here; it just needs to break
    # one wedge per attempt cleanly.
    stuck_ticks = 0
    escape_ticks_remaining = 0
    escape_ang_sign = 0.0
    last_commanded_ang = 0.0
    STUCK_TICKS_THRESHOLD = 30    # ~0.48 s sim of "commanded but stationary"
    ESCAPE_TICKS = 90             # ~1.44 s sim of reverse + unwind
    ESCAPE_LIN_M_S = -0.50

    while robot.step(time_step) != -1:
        # --- Vision export — if a HTTP request bumped vision_request,
        # write a PNG of the operator's viewport before the rest of the
        # tick. exportImage is a Supervisor call, so it must run on the
        # main thread.
        with state.lock:
            req = state.vision_request
            served = state.vision_served
        if req > served:
            try:
                # exportImage takes (filename, quality 1-100); .png ext
                # triggers PNG output regardless of quality.
                robot.exportImage(state.vision_path, 90)
                with state.lock:
                    state.vision_served = req
            except Exception as exc:
                print(f"[husky_omnilink_bridge] exportImage failed: {exc}")
                with state.lock:
                    state.vision_served = req  # don't deadlock the handler

        # --- Handle deferred world reload (kills this process) ---
        with state.lock:
            reload = state.reload_request
            state.reload_request = None
        if reload is not None:
            for m in motors:
                m.setVelocity(0.0)
            try:
                if reload:
                    print(f"[husky_omnilink_bridge] worldLoad({reload!r})")
                    robot.worldLoad(reload)
                else:
                    print("[husky_omnilink_bridge] worldReload()")
                    robot.worldReload()
            except Exception as exc:
                print(f"[husky_omnilink_bridge] reload failed: {exc}")
            # Webots will kill this controller. If we somehow survive,
            # bail out so we don't keep stepping a stale scene.
            return

        # --- Read pose ---
        pos = pose_node.getPosition()
        yaw = yaw_from_orientation(pose_node.getOrientation())
        x, y = float(pos[0]), float(pos[1])

        dt = max(state.tick_period_s, 1e-6)
        v_lin = math.hypot(x - prev_x, y - prev_y) / dt
        v_ang = wrap_pi(yaw - prev_yaw) / dt
        prev_x, prev_y, prev_yaw = x, y, yaw

        goal_reached = math.hypot(x - goal_x, y - goal_y) <= goal_radius

        # --- Handle deferred reset (must run on this thread) ---
        with state.lock:
            reset = state.reset_request
            state.reset_request = None
        if reset is not None and translation_field is not None:
            translation_field.setSFVec3f([reset["x"], reset["y"], 0.15])
            if rotation_field is not None:
                rotation_field.setSFRotation([0.0, 0.0, 1.0, reset["yaw"]])
            try:
                self_node.resetPhysics()
            except Exception:
                pass
            for m in motors:
                m.setVelocity(0.0)
            with state.lock:
                state.task = None
                state.mode = "idle"
                state.left_cmd = 0.0
                state.right_cmd = 0.0
            # Skip motion this tick - we just teleported.
            with state.lock:
                state.x, state.y, state.yaw = reset["x"], reset["y"], reset["yaw"]
                state.v_linear = 0.0
                state.v_angular = 0.0
                state.sim_time += state.tick_period_s
                state.last_tick_at = time.time()
                state.goal_reached = False
            prev_x, prev_y, prev_yaw = reset["x"], reset["y"], reset["yaw"]
            cur_left = cur_right = 0.0
            continue

        # --- Run the active task ---
        with state.lock:
            task = dict(state.task) if state.task else None

        linear = angular = 0.0
        if task is not None:
            linear, angular, done, fault = _twist_for_task(
                task, x, y, yaw, state.sim_time, v_lin, v_ang
            )
            if done:
                with state.lock:
                    if state.task is task or (
                        state.task and state.task.get("kind") == task.get("kind")
                    ):
                        state.task = None
                        state.mode = "idle"
                        if fault:
                            state.fault = fault
                linear = angular = 0.0
                stuck_ticks = 0
                escape_ticks_remaining = 0
            else:
                if escape_ticks_remaining == 0:
                    last_commanded_ang = angular
                commanding = (abs(linear) + abs(angular)) > 0.05
                moving = (abs(v_lin) > 0.05) or (abs(v_ang) > 0.05)
                if escape_ticks_remaining > 0:
                    linear = ESCAPE_LIN_M_S
                    angular = escape_ang_sign * PIVOT_ANGULAR_R_S_MAX
                    escape_ticks_remaining -= 1
                    if escape_ticks_remaining == 0:
                        stuck_ticks = 0
                elif commanding and not moving:
                    stuck_ticks += 1
                    if stuck_ticks >= STUCK_TICKS_THRESHOLD:
                        escape_ticks_remaining = ESCAPE_TICKS
                        if abs(last_commanded_ang) > 0.05:
                            escape_ang_sign = -1.0 if last_commanded_ang > 0 else 1.0
                        else:
                            escape_ang_sign = 1.0
                        linear = ESCAPE_LIN_M_S
                        angular = escape_ang_sign * PIVOT_ANGULAR_R_S_MAX
                else:
                    stuck_ticks = 0
        else:
            stuck_ticks = 0
            escape_ticks_remaining = 0
            last_commanded_ang = 0.0
        # If no task and not stopped, hold zero.

        v_left_target, v_right_target = diff_drive_to_wheel_speeds(linear, angular)
        v_left_target = clamp(v_left_target, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        v_right_target = clamp(v_right_target, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        # Per-tick ramp: limits how fast wheel velocity can change. Stops
        # the diff-drive math from snapping the husky from full forward to
        # full counter-spin in a single tick (which causes wheel slip and
        # the husky wedging against the next wall).
        cur_left += clamp(v_left_target - cur_left, -WHEEL_RAMP_PER_TICK, WHEEL_RAMP_PER_TICK)
        cur_right += clamp(v_right_target - cur_right, -WHEEL_RAMP_PER_TICK, WHEEL_RAMP_PER_TICK)
        for m in left_motors:
            m.setVelocity(cur_left)
        for m in right_motors:
            m.setVelocity(cur_right)

        # Drop a breadcrumb each tick: which cell is the husky in?
        # Only record a cell when the husky is plausibly settled in it
        # (drift < cell_size/3 from centre) so transient pass-throughs
        # during a goto don't count as "explored".
        cur_col_tick = int(round((x - MAZE["origin_x"]) / MAZE["cell_size_m"]))
        cur_row_tick = int(round((y - MAZE["origin_y"]) / MAZE["cell_size_m"]))
        cell_cx = MAZE["origin_x"] + cur_col_tick * MAZE["cell_size_m"]
        cell_cy = MAZE["origin_y"] + cur_row_tick * MAZE["cell_size_m"]
        if (math.hypot(x - cell_cx, y - cell_cy) < MAZE["cell_size_m"] / 3
                and 0 <= cur_col_tick < MAZE["cols"]
                and 0 <= cur_row_tick < MAZE["rows"]):
            with state.lock:
                state.visited_cells.add((cur_col_tick, cur_row_tick))
        with state.lock:
            state.x, state.y, state.yaw = x, y, yaw
            state.v_linear = v_lin
            state.v_angular = v_ang
            state.left_cmd = cur_left
            state.right_cmd = cur_right
            state.sim_time += state.tick_period_s
            state.last_tick_at = time.time()
            state.goal_reached = goal_reached


if __name__ == "__main__":
    main()
