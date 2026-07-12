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

"""Mavic OmniLink bridge — supervisor controller for the drone_survey world.

Mirrors the design of `husky_omnilink_bridge`: a Webots controller that
owns the Mavic 2 Pro's motors + camera + pose, and exposes a small HTTP
surface so an external OmniLink agent can drive the drone. The agent
never imports the Webots `controller` module — it talks JSON over HTTP,
this bridge translates into propeller velocities and gimbal positions.

Wired into the world: chat/omnilink_mavic.wbt's Mavic 2 Pro is a URDFRobot
with supervisor=TRUE and controller="mavic_omnilink_bridge". The
controller runs inside the Mavic's process, so `Supervisor.getSelf()`
returns the Mavic node and we can read its world pose every tick.
The bridge speaks two contracts on the same BridgeState: HTTP /action
for the OmniLink agent, wwi-chat text via the IntentRouter (defined in
mavic_chat_router.py) for the human operator typing into the right-
click Robot Window side panel.

Single-process design (vs. husky's bridge + eye-sidecar split): the
Mavic URDF declares Camera/IMU/GPS via <gazebo><sensor> blocks under
the camera_roll_link / base_link (importer materialises them when
OMNISIM_URDF_USE_SENSORS=1), and the gimbal pitch motor is on the same
Robot, so the bridge can call `getDevice("camera")` directly. The
husky URDF can't host a Camera in the same way for historical importer
reasons, which is why husky_eye exists as a separate Robot. Mavic
doesn't have that constraint — perception lives in the same controller.

URDF caveat — rotor physics: URDF has no `Propeller` analogue, so the
continuous-rotation prop joints spin visually but generate no lift on
their own. The companion module `mavic_dynamics.py` (RotorDynamics)
applies F = k_thrust * ω² per propeller as a body-frame force via
`supervisor.getSelf().addForceWithOffset(...)`, plus a yaw torque
proportional to the diagonal-pair asymmetry, mirroring the
thrustConstants / torqueConstants the legacy PROTO carried.

HTTP surface (default port 6090, loopback only):

    GET  /state
        {
          x, y, z, roll, pitch, yaw,    # world frame, m / rad
          v_xy, v_z,                    # measured ground / vertical speed
          target_altitude_m,            # commanded hover altitude
          gimbal_pitch_rad,             # current camera pitch (0=forward, pi/2=down)
          mode,                         # idle | takeoff | hover | goto | land | landed
          fault,                        # null or short string
          sim_time, last_tick_at,       # seconds, wall-clock epoch
          target,                       # nullable: {x, y, altitude} for goto
          mission_complete              # agent-set via complete_mission
        }

    GET  /capabilities
        {
          robot_id, model,
          mass_kg, max_horizontal_speed_m_s, max_vertical_speed_m_s,
          camera: {width, height, fov_h_rad, fov_v_rad, pitch_range_rad},
          world_title, mission_brief, mission_complete,
          ground_truth_def_names: [...]   # marker DEFs the agent can /solid lookup
        }

    GET  /scan
        Pure-Python analysis of the latest camera frame, projected to
        world coordinates using the drone pose + gimbal pitch + FoV:
        {
          markers: [
            {color, world_x, world_y, fraction, centroid_x_norm, centroid_y_norm,
             distance_m},
            ...
          ],
          frame_summary: {width, height, mean_brightness, frame_age_s},
          pose: {x, y, z, yaw, gimbal_pitch_rad},
        }

    GET  /image
        Latest camera frame as base64 PNG. Fallback / debugging only —
        agents should prefer /scan (perception-as-tool, ~150x cheaper).

    GET  /solid?def=NAME
        Ground-truth pose of any DEF'd Solid in the scene. Used by the
        agent (or by tests) to verify a /scan detection is actually where
        the bridge says it is.

    GET  /mission
        Mission brief (free-form text from WorldInfo.info). The agent
        interprets it; the bridge does not score it.

    POST /action  {"action": "...", ...params}
        takeoff             - {altitude} m; spool motors and climb to altitude
        land                - cut to a controlled descent, then idle motors
        hover               - hold current xy + altitude
        goto_waypoint       - {x, y, altitude?, wait?, timeout_s?, yaw_to?}
                              fly to (x, y) at altitude (default = current).
                              wait=true blocks until arrival (or timeout).
                              yaw_to=[lx, ly] aims the nose at (lx, ly) on arrival.
        set_gimbal_pitch    - {pitch_rad} 0=forward, pi/2=straight down
        set_yaw             - {yaw_rad} rotate body to absolute world yaw
        stop                - cut all attitude inputs, free-fall the props
                              (use only as last resort)
        reset               - teleport back to start pose, mission_complete=false
        complete_mission    - {rationale, payload?} log mission completion;
                              mission_complete -> true. Bridge does not score.

The bridge does primitive flight only. Mission planning, waypoint pattern
selection, and red-marker counting all live in the agent prompt.
"""

from __future__ import annotations

import argparse as _argparse
import base64
import json
import math
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

from omnisim import Supervisor

from mavic_dynamics import RotorDynamics
from mavic_chat_router import IntentRouter, handle_wwi_message, push_configure


# Mirror stdout/stderr to a tempfile so we can debug from outside Webots.
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
                                  f"mavic_omnilink_bridge_{_os_for_log.getpid()}.log")
try:
    _log_fp = open(_log_path, "w", encoding="utf-8", buffering=1)
    _sys_for_log.stdout = _Tee(_sys_for_log.__stdout__, _log_fp)
    _sys_for_log.stderr = _Tee(_sys_for_log.__stderr__, _log_fp)
    print(f"[mavic_omnilink_bridge] log file: {_log_path}")
except Exception:
    pass


# --- Mavic flight constants (calibrated empirically from mavic2pro.py) -----
# Same values the stock C / Python controllers use — tuned for the Mavic2Pro
# proto's mass + propeller geometry. Don't rebalance these without verifying
# stable hover first; a 2% change in K_VERTICAL_THRUST tips the system.

K_VERTICAL_THRUST = 68.5   # propeller thrust at hover
K_VERTICAL_OFFSET = 0.6    # vertical PID setpoint offset
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

MAX_YAW_DISTURBANCE = 0.4
MAX_PITCH_DISTURBANCE = -1.0   # negative = pitch nose down to fly forward

# Approach-target precision (meters).
WAYPOINT_REACH_TOL_M = 0.6
ALTITUDE_REACH_TOL_M = 0.4

# Gimbal pitch motor range (camera pitch). 0 = forward, pi/2 (~1.5708) = down.
GIMBAL_PITCH_MIN = -0.5
GIMBAL_PITCH_MAX = 1.7
GIMBAL_DOWN_RAD = math.pi / 2

# Default takeoff / cruise altitude.
DEFAULT_TAKEOFF_ALTITUDE = 12.0


# --- Bridge config ---------------------------------------------------------

BRIDGE_HOST = "127.0.0.1"


def _parse_bridge_args() -> int:
    parser = _argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", type=int, default=6090)
    parser.add_argument("--name", default="mavic2pro")
    args, _unknown = parser.parse_known_args()
    return args.port


BRIDGE_PORT = _parse_bridge_args()


# --- Math / image helpers --------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def encode_png(width: int, height: int, bgra_bytes: bytes) -> bytes:
    """BGRA -> RGB PNG. No PIL dependency. Same encoder as husky_eye."""
    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        b = bgra_bytes[i * 4 + 0]
        g = bgra_bytes[i * 4 + 1]
        r = bgra_bytes[i * 4 + 2]
        rgb[i * 3 + 0] = r
        rgb[i * 3 + 1] = g
        rgb[i * 3 + 2] = b
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
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


# --- Perception: pure-Python BGRA -> structured marker digest --------------

def _classify_blobs(bgra_bytes: bytes, w: int, h: int) -> dict:
    """Walk the BGRA frame once and return per-colour {pixels, sum_x, sum_y,
    fraction, centroid_norm}. Threshold and gates match husky_eye._analyze_bgra
    so the demo's perception story is consistent across robots.

    Six recognised colours: red, green, blue, yellow, magenta, cyan. The
    drone_survey world also has orange + white markers as distractors —
    they get classified as the nearest colour (red / yellow / cyan blends
    or "other"). Only colour blobs above a noise floor are returned.
    """
    total = w * h
    if len(bgra_bytes) < total * 4:
        return {"error": "frame buffer too small"}

    counts = {"red": 0, "green": 0, "blue": 0,
              "yellow": 0, "magenta": 0, "cyan": 0}
    sum_x = {k: 0 for k in counts}
    sum_y = {k: 0 for k in counts}
    bright_sum = 0

    raw = bgra_bytes
    for y in range(h):
        row_off = y * w * 4
        for x in range(w):
            i = row_off + x * 4
            B = raw[i]
            G = raw[i + 1]
            R = raw[i + 2]
            bright_sum += R + G + B
            tag = None
            if R > 140 and G < 90 and B < 90:
                tag = "red"
            elif G > 140 and R < 90 and B < 90:
                tag = "green"
            elif B > 140 and R < 90 and G < 90:
                tag = "blue"
            elif R > 140 and G > 130 and B < 90:
                tag = "yellow"
            elif R > 140 and B > 130 and G < 90:
                tag = "magenta"
            elif G > 140 and B > 140 and R < 90:
                tag = "cyan"
            if tag is not None:
                counts[tag] += 1
                sum_x[tag] += x
                sum_y[tag] += y

    # Threshold: 0.05 % of frame area. Drone_survey markers are 0.8 m
    # squares viewed from 12 m — they subtend ~60 px in a 400×240 frame
    # (~0.6 % of total pixels), comfortably above noise. The threshold
    # is conservative enough that distant markers in oblique views still
    # register but stray red pixels from the warehouse roof or shadows
    # don't generate false detections.
    threshold = total * 0.0005
    blobs = []
    for c, n in counts.items():
        if n <= threshold:
            continue
        cx = sum_x[c] / n
        cy = sum_y[c] / n
        blobs.append({
            "color": c,
            "pixels": n,
            "fraction": round(n / total, 4),
            "centroid_x_norm": round(cx / max(w - 1, 1), 4),
            "centroid_y_norm": round(cy / max(h - 1, 1), 4),
        })
    return {
        "blobs": blobs,
        "mean_brightness": round(bright_sum / (total * 3), 1),
    }


def _project_to_world(centroid_x_norm: float, centroid_y_norm: float,
                       drone_x: float, drone_y: float, drone_alt: float,
                       drone_yaw: float, gimbal_pitch_rad: float,
                       fov_h_rad: float, fov_v_rad: float) -> dict:
    """Map an image-space centroid to world (x, y) for a downward-pointing
    camera. Assumes the gimbal is close to straight down (pitch ≈ pi/2)
    and the ground is z = 0.

    Geometry: centroid_x_norm/y_norm is (0,0)=top-left, (1,1)=bottom-right.
    Image x grows to the right, y grows down. With the camera pointed
    straight down and the drone facing +X (yaw 0), pixel +x maps to body
    +Y (left in body frame becomes +Y in world after yaw rotation), pixel
    -y (up in image) maps to body +X (forward).

    For pitch != pi/2, the projection skews along the camera's optical
    axis. We use a small-angle approximation: deviations of ±0.1 rad from
    pi/2 introduce <1 m error at 12 m altitude, which is well under the
    1.5 m dedup tolerance the agent uses. For larger deviations the
    projection becomes meaningfully wrong and the caller should re-take
    the scan with the gimbal closer to straight down.
    """
    # Image-frame angular offset from camera optical axis.
    # x_norm=0.5 is the optical axis (no offset). +x_norm => +angle right
    # (in camera body frame). +y_norm => +angle down (in camera body frame).
    half_fov_h = fov_h_rad / 2.0
    half_fov_v = fov_v_rad / 2.0
    # tan(angle) = (centroid - 0.5) / 0.5 * tan(half_fov)
    tan_h = (centroid_x_norm - 0.5) * 2.0 * math.tan(half_fov_h)
    tan_v = (centroid_y_norm - 0.5) * 2.0 * math.tan(half_fov_v)
    # Distance from drone to ground point along camera optical axis.
    # When gimbal is straight down, this is just drone altitude.
    # For oblique angles (pitch < pi/2), the slant range increases.
    sin_pitch = math.sin(gimbal_pitch_rad)
    if sin_pitch < 0.05:
        # Camera is nearly horizontal — projection is undefined / very
        # noisy. Punt with NaN and let the caller decide.
        return {"world_x": float("nan"), "world_y": float("nan"),
                "distance_m": float("nan"), "valid": False}
    # Body-frame ground offsets (+x = forward, +y = left).
    # With camera pointed down: image-up = body-forward, image-right = body-right.
    body_dx = -tan_v * drone_alt / sin_pitch     # forward (image-up = forward)
    body_dy = -tan_h * drone_alt / sin_pitch     # left  (image-right = body-right = -y)
    # Rotate body-frame into world-frame using yaw.
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    world_dx = body_dx * cos_y - body_dy * sin_y
    world_dy = body_dx * sin_y + body_dy * cos_y
    world_x = drone_x + world_dx
    world_y = drone_y + world_dy
    distance = math.hypot(world_dx, world_dy) + drone_alt  # rough slant range
    return {
        "world_x": round(world_x, 2),
        "world_y": round(world_y, 2),
        "distance_m": round(distance, 2),
        "valid": True,
    }


# --- Shared state ----------------------------------------------------------

class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Velocities — derived from successive poses each tick.
        self.last_pose_for_v = None  # (x, y, z, t)
        self.v_xy = 0.0
        self.v_z = 0.0
        # Commanded setpoints (consumed by the flight loop).
        self.target_altitude = 0.0
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.gimbal_pitch_target = GIMBAL_DOWN_RAD
        self.gimbal_pitch_actual = 0.0
        self.mode = "idle"        # idle | takeoff | hover | goto | land | landed
        self.fault: Optional[str] = None
        self.sim_time = 0.0
        self.last_tick_at = time.time()
        self.tick_period_s = 0.008
        # Camera frame buffer (latest BGRA + timestamp). Refreshed when the
        # /scan or /image handler asks for it (on-demand capture keeps the
        # main loop unblocked when nobody is watching).
        self.camera_w = 0
        self.camera_h = 0
        self.camera_fov_h = 0.0
        self.camera_fov_v = 0.0
        self.frame_request = 0
        self.frame_served = 0
        self.frame_bgra: Optional[bytes] = None
        self.frame_pose = (0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, z, yaw, gimbal_pitch
        self.frame_sim_time = 0.0
        # Mission state.
        self.world_title = ""
        self.mission_brief = ""
        self.mission_complete = False
        self.mission_log = []        # list of {ts, sim_time, rationale, payload}
        # Reset request — main loop teleports the drone back to start.
        self.reset_request: Optional[dict] = None
        # Ground-truth-able DEFs in the world (advertised in /capabilities).
        self.gt_def_names: list = []
        # wwi-chat outbox: lines the main loop should push to the robot
        # window (configure / status / agent / tool). Populated by the
        # chat router; drained in main() each tick.
        self.window_outbox: list = []
        self.window_configured = False

    def snapshot(self) -> dict:
        with self.lock:
            target = None
            if self.target_x is not None and self.target_y is not None:
                target = {
                    "x": self.target_x, "y": self.target_y,
                    "altitude": self.target_altitude,
                }
            return {
                "x": self.x, "y": self.y, "z": self.z,
                "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
                "v_xy": self.v_xy, "v_z": self.v_z,
                "target_altitude_m": self.target_altitude,
                "gimbal_pitch_rad": self.gimbal_pitch_actual,
                "gimbal_pitch_target_rad": self.gimbal_pitch_target,
                "mode": self.mode,
                "fault": self.fault,
                "sim_time": self.sim_time,
                "last_tick_at": self.last_tick_at,
                "target": target,
                "mission_complete": self.mission_complete,
            }


# --- Synchronous-mode helper (mirrors husky bridge's _wait_for_task_idle) --

def _wait_until_arrived(state: BridgeState, target_x: float, target_y: float,
                         target_alt: float, timeout_s: float,
                         poll_s: float = 0.05) -> dict:
    """Block until the drone is within tolerance of (target_x, target_y, target_alt),
    or until the wall-clock deadline expires. Returns
    {done, fault, x, y, z, yaw, sim_time, distance_remaining_m}.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        with state.lock:
            f = state.fault
            x, y, z, yaw, st = state.x, state.y, state.z, state.yaw, state.sim_time
        if f:
            return {"done": False, "fault": f, "x": x, "y": y, "z": z, "yaw": yaw,
                    "sim_time": st, "distance_remaining_m": math.hypot(x - target_x, y - target_y)}
        d_xy = math.hypot(x - target_x, y - target_y)
        d_z = abs(z - target_alt)
        if d_xy < WAYPOINT_REACH_TOL_M and d_z < ALTITUDE_REACH_TOL_M:
            return {"done": True, "fault": None, "x": x, "y": y, "z": z, "yaw": yaw,
                    "sim_time": st, "distance_remaining_m": d_xy}
        time.sleep(poll_s)
    with state.lock:
        f = state.fault
        x, y, z, yaw, st = state.x, state.y, state.z, state.yaw, state.sim_time
    return {"done": False, "fault": f or "goto_waypoint_timeout",
            "x": x, "y": y, "z": z, "yaw": yaw, "sim_time": st,
            "distance_remaining_m": math.hypot(x - target_x, y - target_y)}


# --- HTTP handlers ---------------------------------------------------------

def make_handler(state: BridgeState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            try:
                body = json.dumps(obj, allow_nan=False).encode("utf-8")
            except ValueError:
                body = json.dumps({
                    "error": "non-finite value in response",
                    "hint": "drone may not have settled yet",
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

        def _request_frame(self) -> Optional[dict]:
            """Bump frame_request, wait briefly for the main loop to refresh
            the buffer, return the captured frame + pose. Returns None on
            timeout."""
            with state.lock:
                state.frame_request += 1
                req_id = state.frame_request
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with state.lock:
                    if state.frame_served >= req_id and state.frame_bgra:
                        return {
                            "bgra": state.frame_bgra,
                            "pose": state.frame_pose,
                            "sim_time": state.frame_sim_time,
                            "w": state.camera_w,
                            "h": state.camera_h,
                            "fov_h": state.camera_fov_h,
                            "fov_v": state.camera_fov_v,
                        }
                time.sleep(0.02)
            return None

        # ----- GET endpoints ----------------------------------------------

        def do_GET(self):
            if self.path == "/state":
                self._json(200, state.snapshot())
                return
            if self.path == "/capabilities":
                self._json(200, {
                    "robot_id": "mavic2pro",
                    "model": "DJI Mavic 2 Pro",
                    "mass_kg": 0.9,                  # ~real Mavic 2 Pro mass
                    "max_horizontal_speed_m_s": 18,  # spec sport-mode top speed
                    "max_vertical_speed_m_s": 5,
                    "default_takeoff_altitude_m": DEFAULT_TAKEOFF_ALTITUDE,
                    "camera": {
                        "width": state.camera_w,
                        "height": state.camera_h,
                        "fov_h_rad": state.camera_fov_h,
                        "fov_v_rad": state.camera_fov_v,
                        "pitch_range_rad": [GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX],
                        "down_pitch_rad": GIMBAL_DOWN_RAD,
                    },
                    "world_title": state.world_title,
                    "mission_brief": state.mission_brief,
                    "mission_complete": state.mission_complete,
                    "ground_truth_def_names": list(state.gt_def_names),
                    "perception_hint": (
                        "Prefer /scan over /image — /scan returns structured "
                        "marker positions in world coordinates (~150x cheaper "
                        "than sending pixels to the LLM). /image is fallback "
                        "for genuinely ambiguous frames."
                    ),
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
                        "When the mission is satisfied POST /action "
                        "{action:'complete_mission', rationale:'<one sentence>', "
                        "payload:{...}}. The bridge logs your claim, it does "
                        "not score it. Operators audit the log."
                    ),
                })
                return
            if self.path == "/scan":
                frame = self._request_frame()
                if frame is None:
                    self._json(504, {"error": "timed out waiting for camera frame"})
                    return
                analysis = _classify_blobs(frame["bgra"], frame["w"], frame["h"])
                if "error" in analysis:
                    self._json(500, analysis)
                    return
                fx, fy, fz, fyaw, fpitch = frame["pose"]
                markers = []
                for blob in analysis["blobs"]:
                    proj = _project_to_world(
                        blob["centroid_x_norm"], blob["centroid_y_norm"],
                        fx, fy, fz, fyaw, fpitch,
                        frame["fov_h"], frame["fov_v"],
                    )
                    markers.append({
                        "color": blob["color"],
                        "pixels": blob["pixels"],
                        "fraction": blob["fraction"],
                        "centroid_x_norm": blob["centroid_x_norm"],
                        "centroid_y_norm": blob["centroid_y_norm"],
                        "world_x": proj["world_x"],
                        "world_y": proj["world_y"],
                        "distance_m": proj["distance_m"],
                        "projection_valid": proj["valid"],
                    })
                # Sort by world distance so the closest detection (most
                # reliable projection) shows up first.
                markers.sort(key=lambda m: (
                    not m["projection_valid"],
                    m.get("distance_m") or 1e9,
                ))
                self._json(200, {
                    "markers": markers,
                    "frame_summary": {
                        "width": frame["w"],
                        "height": frame["h"],
                        "mean_brightness": analysis["mean_brightness"],
                        "frame_age_s": round(state.sim_time - frame["sim_time"], 3),
                    },
                    "pose": {
                        "x": fx, "y": fy, "z": fz, "yaw": fyaw,
                        "gimbal_pitch_rad": fpitch,
                    },
                    "hint": (
                        "world_x/world_y are the projected ground positions of "
                        "each detected colour blob. Aggregate detections from "
                        "multiple waypoints by deduplicating within ~1.5 m "
                        "(detections of the same marker from different vantages "
                        "should cluster within that radius)."
                    ),
                })
                return
            if self.path == "/image":
                frame = self._request_frame()
                if frame is None:
                    self._json(504, {"error": "timed out waiting for camera frame"})
                    return
                png = encode_png(frame["w"], frame["h"], frame["bgra"])
                fx, fy, fz, fyaw, fpitch = frame["pose"]
                self._json(200, {
                    "width": frame["w"],
                    "height": frame["h"],
                    "encoding": "image/png; base64",
                    "image_base64": base64.b64encode(png).decode("ascii"),
                    "pose": {
                        "x": fx, "y": fy, "z": fz, "yaw": fyaw,
                        "gimbal_pitch_rad": fpitch,
                    },
                    "sim_time": frame["sim_time"],
                })
                return
            if self.path.startswith("/solid"):
                from urllib.parse import urlparse, parse_qs as _parse_qs
                qs = _parse_qs(urlparse(self.path).query)
                def_name = (qs.get("def") or [""])[0].strip()
                if not def_name:
                    self._json(400, {
                        "error": "missing 'def' query param",
                        "example": "/solid?def=MARKER_RED_1",
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
                        "hint": (
                            "names are case-sensitive; this world advertises: "
                            + ", ".join(state.gt_def_names)
                        ),
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
            self._json(404, {"error": "not found"})

        # ----- POST /action ----------------------------------------------

        def do_POST(self):
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
                    state.target_x = None
                    state.target_y = None
                    state.target_yaw = None
                    state.target_altitude = 0.0
                    state.mode = "idle"
                    state.fault = None
                self._ok({"halted_at": time.time()})
                return

            if action == "reset":
                with state.lock:
                    state.target_x = None
                    state.target_y = None
                    state.target_yaw = None
                    state.target_altitude = 0.0
                    state.mode = "idle"
                    state.fault = None
                    state.mission_complete = False
                    state.mission_log = []
                    state.reset_request = {"x": 0.0, "y": -12.0, "z": 0.1, "yaw": math.pi / 2}
                self._ok({"reset": True})
                return

            if action == "complete_mission":
                rationale = (body.get("rationale") or "").strip()
                payload = body.get("payload") or {}
                if not rationale:
                    self._json(400, {"error": "rationale is required (one-sentence why)"})
                    return
                ts = time.time()
                with state.lock:
                    state.mission_complete = True
                    state.mission_log.append({
                        "timestamp": ts,
                        "sim_time": state.sim_time,
                        "rationale": rationale,
                        "payload": payload,
                    })
                self._ok({
                    "mission_complete": True,
                    "logged_at": ts,
                    "log_entry": state.mission_log[-1],
                })
                return

            if action == "takeoff":
                altitude = float(body.get("altitude", DEFAULT_TAKEOFF_ALTITUDE))
                with state.lock:
                    state.target_altitude = max(0.5, altitude)
                    state.target_x = state.x
                    state.target_y = state.y
                    state.mode = "takeoff"
                    state.fault = None
                if body.get("wait"):
                    timeout_s = float(body.get("timeout_s", 30.0))
                    res = _wait_until_arrived(state, state.x, state.y, altitude, timeout_s)
                    self._ok({"target_altitude": altitude, "waited": True, **res})
                    return
                self._ok({"target_altitude": altitude})
                return

            if action == "land":
                with state.lock:
                    state.target_x = state.x
                    state.target_y = state.y
                    state.target_altitude = 0.0
                    state.mode = "land"
                    state.fault = None
                if body.get("wait"):
                    timeout_s = float(body.get("timeout_s", 30.0))
                    deadline = time.time() + timeout_s
                    while time.time() < deadline:
                        with state.lock:
                            z = state.z
                            f = state.fault
                            mode = state.mode
                        if f:
                            self._ok({"waited": True, "landed": False, "fault": f, "z": z})
                            return
                        if z < 0.30 and mode == "landed":
                            self._ok({"waited": True, "landed": True, "z": z})
                            return
                        time.sleep(0.1)
                    self._ok({"waited": True, "landed": False, "fault": "land_timeout"})
                    return
                self._ok({"target_altitude": 0.0})
                return

            if action == "hover":
                with state.lock:
                    state.target_x = state.x
                    state.target_y = state.y
                    if state.target_altitude < 0.5:
                        state.target_altitude = max(state.z, DEFAULT_TAKEOFF_ALTITUDE)
                    state.mode = "hover"
                    state.fault = None
                self._ok({"hovering_at": {"x": state.x, "y": state.y, "z": state.z}})
                return

            if action == "goto_waypoint":
                try:
                    tx = float(body.get("x"))
                    ty = float(body.get("y"))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad x/y: {exc}"})
                    return
                with state.lock:
                    target_alt = float(body.get("altitude", state.target_altitude or DEFAULT_TAKEOFF_ALTITUDE))
                    state.target_x = tx
                    state.target_y = ty
                    state.target_altitude = target_alt
                    state.mode = "goto"
                    state.fault = None
                if body.get("wait"):
                    timeout_s = float(body.get("timeout_s", 60.0))
                    res = _wait_until_arrived(state, tx, ty, target_alt, timeout_s)
                    # Optional yaw_to: aim nose at (lx, ly) on arrival.
                    yaw_to = body.get("yaw_to")
                    if (res["done"] and not res["fault"]
                            and isinstance(yaw_to, (list, tuple)) and len(yaw_to) == 2):
                        try:
                            lx = float(yaw_to[0]); ly = float(yaw_to[1])
                            desired_yaw = wrap_pi(math.atan2(ly - res["y"], lx - res["x"]))
                            with state.lock:
                                state.target_yaw = desired_yaw
                            time.sleep(0.5)  # let yaw settle
                            with state.lock:
                                res["yaw"] = state.yaw
                        except Exception:
                            pass
                    self._ok({"x": tx, "y": ty, "altitude": target_alt, "waited": True, **res})
                    return
                self._ok({"x": tx, "y": ty, "altitude": target_alt})
                return

            if action == "set_gimbal_pitch":
                try:
                    pitch = float(body.get("pitch_rad"))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad pitch_rad: {exc}"})
                    return
                pitch = clamp(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX)
                with state.lock:
                    state.gimbal_pitch_target = pitch
                self._ok({"gimbal_pitch_target_rad": pitch})
                return

            if action == "set_yaw":
                try:
                    yaw = float(body.get("yaw_rad"))
                except (TypeError, ValueError) as exc:
                    self._json(400, {"error": f"bad yaw_rad: {exc}"})
                    return
                with state.lock:
                    state.target_yaw = wrap_pi(yaw)
                self._ok({"target_yaw_rad": wrap_pi(yaw)})
                return

            self._json(400, {
                "error": f"unknown action {action!r}",
                "actions": ["takeoff", "land", "hover", "goto_waypoint",
                            "set_gimbal_pitch", "set_yaw", "stop", "reset",
                            "complete_mission"],
            })

    return Handler


# --- Webots-side scaffolding -----------------------------------------------

def _collect_marker_defs(supervisor) -> list:
    """Walk the world for Solid nodes whose name starts with 'marker_'.
    Returns the DEF names so /capabilities can advertise what's
    /solid-lookup-able (mirrors the husky bridge's gt_def_names contract).
    """
    root = supervisor.getRoot()
    children_field = root.getField("children") if root is not None else None
    defs = []
    if children_field is None:
        return defs
    try:
        count = children_field.getCount()
    except Exception:
        return defs
    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            name_f = node.getField("name")
            if name_f is None:
                continue
            name = name_f.getSFString()
        except Exception:
            continue
        if name and (name.startswith("marker_") or name == "warehouse"):
            try:
                def_name = node.getDef()
                if def_name:
                    defs.append(def_name)
            except Exception:
                pass
    return defs


def _read_world_brief(supervisor) -> tuple:
    """Pull WorldInfo.title + WorldInfo.info as the operator-visible brief."""
    root = supervisor.getRoot()
    children_field = root.getField("children") if root is not None else None
    if children_field is None:
        return ("", "")
    try:
        count = children_field.getCount()
    except Exception:
        return ("", "")
    title = ""
    info_lines = []
    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            type_name = node.getTypeName()
        except Exception:
            continue
        if type_name == "WorldInfo":
            try:
                title = node.getField("title").getSFString()
            except Exception:
                pass
            try:
                info_field = node.getField("info")
                n_lines = info_field.getCount()
                for j in range(n_lines):
                    info_lines.append(info_field.getMFString(j))
            except Exception:
                pass
            break
    return (title, "\n".join(info_lines))


def main():
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(encoding="utf-8", errors="backslashreplace")
        except Exception:
            pass

    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    self_node = supervisor.getSelf()
    if self_node is None:
        print("[mavic_omnilink_bridge] ERROR: getSelf() returned None — supervisor TRUE missing?")
        return

    # Enable sensors.
    camera = supervisor.getDevice("camera")
    if camera is None:
        print("[mavic_omnilink_bridge] ERROR: no 'camera' device — Mavic2Pro cameraSlot empty?")
        return
    camera.enable(time_step)
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()
    cam_fov_h = camera.getFov()
    # Approximate vertical FoV from horizontal FoV + aspect ratio. Webots
    # cameras report the horizontal FoV; vertical = 2 * atan(tan(fov_h/2) * h/w).
    cam_fov_v = 2.0 * math.atan(math.tan(cam_fov_h / 2.0) * (cam_h / cam_w))
    print(f"[mavic_omnilink_bridge] camera: {cam_w}x{cam_h} fov_h={cam_fov_h:.2f} fov_v={cam_fov_v:.2f}")

    imu = supervisor.getDevice("inertial unit")
    gps = supervisor.getDevice("gps")
    gyro = supervisor.getDevice("gyro")
    for d in (imu, gps, gyro):
        if d is None:
            print("[mavic_omnilink_bridge] ERROR: missing IMU/GPS/Gyro on Mavic")
            return
        d.enable(time_step)

    # Propeller motors.
    front_left = supervisor.getDevice("front left propeller")
    front_right = supervisor.getDevice("front right propeller")
    rear_left = supervisor.getDevice("rear left propeller")
    rear_right = supervisor.getDevice("rear right propeller")
    motors = [front_left, front_right, rear_left, rear_right]
    if any(m is None for m in motors):
        print("[mavic_omnilink_bridge] ERROR: missing propeller motors")
        return
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(1.0)

    # URDF prop joints don't generate lift on their own. Drive the body
    # forces from supervisor add_force_with_offset each tick, using the
    # same motor target velocities the PID below computes.
    dynamics = RotorDynamics(self_node)

    # Gimbal pitch motor — drives the camera angle. Default to straight
    # down so /scan's world projection works out of the box.
    gimbal_pitch_motor = supervisor.getDevice("camera pitch")
    if gimbal_pitch_motor is None:
        print("[mavic_omnilink_bridge] WARN: no 'camera pitch' device — gimbal control disabled")
    else:
        gimbal_pitch_motor.setPosition(GIMBAL_DOWN_RAD)

    # Pose-tracking field (translation/rotation) for reset_request.
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    initial_translation = list(translation_field.getSFVec3f()) if translation_field else [0.0, 0.0, 0.1]
    initial_rotation = list(rotation_field.getSFRotation()) if rotation_field else [0.0, 0.0, 1.0, 0.0]

    state = BridgeState()
    state.supervisor = supervisor   # type: ignore[attr-defined]
    state.tick_period_s = time_step / 1000.0
    state.camera_w = cam_w
    state.camera_h = cam_h
    state.camera_fov_h = cam_fov_h
    state.camera_fov_v = cam_fov_v
    state.gimbal_pitch_target = GIMBAL_DOWN_RAD
    state.gimbal_pitch_actual = GIMBAL_DOWN_RAD

    # Seed pose so /state is sane before the first tick.
    state.x = initial_translation[0]
    state.y = initial_translation[1]
    state.z = initial_translation[2]

    # Mission brief / world title.
    title, brief = _read_world_brief(supervisor)
    state.world_title = title
    state.mission_brief = brief

    # Discoverable ground-truth DEFs.
    state.gt_def_names = _collect_marker_defs(supervisor)
    print(f"[mavic_omnilink_bridge] world_title={title!r}")
    print(f"[mavic_omnilink_bridge] gt_defs: {state.gt_def_names}")

    # Start HTTP server.
    server = ThreadingHTTPServer((BRIDGE_HOST, BRIDGE_PORT), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[mavic_omnilink_bridge] HTTP listening on http://{BRIDGE_HOST}:{BRIDGE_PORT}")

    # Chat-window IntentRouter (right-click *Show Robot Window* → side panel).
    chat_router = IntentRouter(state)

    # Stabiliser PID — adapted from the stock mavic2pro.py controller.
    # The flight-loop math runs unchanged from the reference controller; we
    # add the bridge's per-mode setpoint logic on top of it.

    def _flight_step():
        roll, pitch, yaw = imu.getRollPitchYaw()
        x_pos, y_pos, altitude = gps.getValues()
        roll_acc, pitch_acc, _ = gyro.getValues()

        # Update measured velocities from successive poses.
        now = time.time()
        prev = state.last_pose_for_v
        if prev is not None:
            dt = now - prev[3]
            if dt > 1e-3:
                state.v_xy = math.hypot(x_pos - prev[0], y_pos - prev[1]) / dt
                state.v_z = (altitude - prev[2]) / dt
        state.last_pose_for_v = (x_pos, y_pos, altitude, now)

        with state.lock:
            state.x = x_pos
            state.y = y_pos
            state.z = altitude
            state.roll = roll
            state.pitch = pitch
            state.yaw = yaw
            state.last_tick_at = now
            state.sim_time += time_step / 1000.0
            target_x = state.target_x
            target_y = state.target_y
            target_altitude = state.target_altitude
            target_yaw = state.target_yaw
            mode = state.mode

        # Mode-specific setpoint computation.
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        if mode in ("takeoff", "hover", "goto", "land") and target_altitude > 0.05:
            # Climb until at altitude before touching xy.
            if altitude > target_altitude - 1.0 and target_x is not None and target_y is not None:
                # Heading-to-target in world frame.
                dx = target_x - x_pos
                dy = target_y - y_pos
                dist_xy = math.hypot(dx, dy)
                if dist_xy > WAYPOINT_REACH_TOL_M:
                    desired_heading = math.atan2(dy, dx)
                    angle_left = wrap_pi(desired_heading - yaw)
                    yaw_disturbance = MAX_YAW_DISTURBANCE * angle_left / (2.0 * math.pi)
                    pitch_disturbance = clamp(
                        math.log10(abs(angle_left) + 1e-3),
                        MAX_PITCH_DISTURBANCE, 0.1,
                    )
                    if mode == "takeoff":
                        with state.lock:
                            state.mode = "goto"
                else:
                    # Arrived — switch to hover. yaw disturbance from optional set_yaw.
                    if target_yaw is not None:
                        ye = wrap_pi(target_yaw - yaw)
                        yaw_disturbance = MAX_YAW_DISTURBANCE * ye / (2.0 * math.pi)
                    if mode in ("goto", "takeoff"):
                        with state.lock:
                            state.mode = "hover"
            elif target_yaw is not None:
                # Climbing to altitude but operator set a yaw — still apply it.
                ye = wrap_pi(target_yaw - yaw)
                yaw_disturbance = MAX_YAW_DISTURBANCE * ye / (2.0 * math.pi)

        if mode == "land":
            # Bring vertical setpoint smoothly to ground; below 0.4 m -> idle motors.
            if altitude < 0.4 and abs(state.v_z) < 0.3:
                with state.lock:
                    state.mode = "landed"
                    state.target_altitude = 0.0
                for m in motors:
                    m.setVelocity(0.0)
                return

        # Stabiliser PID (verbatim from mavic2pro.py — don't rebalance).
        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_acc + roll_disturbance
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_acc + pitch_disturbance
        yaw_input = yaw_disturbance
        # Read target_altitude AFTER the land branch may have set it to 0.
        with state.lock:
            t_alt = state.target_altitude
        clamped_diff_alt = clamp(t_alt - altitude + K_VERTICAL_OFFSET, -1, 1)
        vertical_input = K_VERTICAL_P * (clamped_diff_alt ** 3.0)

        front_left_input = K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
        front_right_input = K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
        rear_left_input = K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
        rear_right_input = K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

        # Idle motors when the bridge is purely passive (no takeoff requested).
        if mode == "idle" or mode == "landed":
            for m in motors:
                m.setVelocity(0.0)
            return

        front_left.setVelocity(front_left_input)
        front_right.setVelocity(-front_right_input)
        rear_left.setVelocity(-rear_left_input)
        rear_right.setVelocity(rear_right_input)

        # Replace the Propeller-node lift/torque the legacy PROTO did
        # automatically. Pass raw signed inputs — RotorDynamics squares
        # for thrust and uses pair asymmetry for yaw torque.
        dynamics.step(front_left_input, front_right_input, rear_left_input, rear_right_input)

        # Gimbal: drive the pitch motor toward the current target.
        if gimbal_pitch_motor is not None:
            with state.lock:
                tgt = state.gimbal_pitch_target
            gimbal_pitch_motor.setPosition(tgt)
            with state.lock:
                state.gimbal_pitch_actual = tgt

    while supervisor.step(time_step) != -1:
        # Honour a queued reset (only the main thread is allowed to teleport).
        with state.lock:
            req = state.reset_request
            state.reset_request = None
        if req and translation_field and rotation_field:
            translation_field.setSFVec3f([req["x"], req["y"], req.get("z", 0.1)])
            rotation_field.setSFRotation([0.0, 0.0, 1.0, req.get("yaw", math.pi / 2)])
            with state.lock:
                state.last_pose_for_v = None
                state.v_xy = 0.0
                state.v_z = 0.0

        _flight_step()

        # Drain wwi inbox (robot-window messages from the chat side panel).
        while True:
            msg = supervisor.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(state, chat_router, msg)
            except Exception as exc:
                with state.lock:
                    state.window_outbox.append(f"error:chat_router_exception: {exc!r}")

        # Flush window outbox (configure / status / agent / tool lines).
        with state.lock:
            outbox = state.window_outbox
            state.window_outbox = []
        for line in outbox:
            try:
                supervisor.wwiSendText(line)
            except Exception as exc:
                print(f"[mavic_omnilink_bridge] wwiSendText failed: {exc}")

        # Honour camera capture requests (cheap — Webots' getImage reuses
        # the buffer the renderer already produced this tick).
        with state.lock:
            req = state.frame_request
            served = state.frame_served
        if req > served:
            try:
                raw = camera.getImage()
                if raw:
                    with state.lock:
                        state.frame_bgra = raw
                        state.frame_pose = (state.x, state.y, state.z,
                                             state.yaw, state.gimbal_pitch_actual)
                        state.frame_sim_time = state.sim_time
                        state.frame_served = req
                else:
                    with state.lock:
                        state.frame_served = req
            except Exception as exc:
                print(f"[mavic_omnilink_bridge] camera capture failed: {exc}")
                with state.lock:
                    state.frame_served = req


if __name__ == "__main__":
    main()
