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

"""omnilink_mobile_bridge — generic OmniLink bridge for URDF wheeled bases.

Drop this in as the URDFRobot's controller (`supervisor TRUE`):

    controller "omnilink_mobile_bridge"
    controllerArgs ["--robot" "husky" "--port" "8765"]

Picks a config from `_mobile_configs.py` and drives the URDF's wheel
motors with a unified diff-drive / skid-steer surface. Adding a new
base: append a config and (if needed) a new wheel-motor layout to
WHEEL_MOTORS.

Surfaces
--------
HTTP on 127.0.0.1:<port> (default 8765), Axis-normalized:

    POST /list_robots         -> [{id, model, capabilities}]
    POST /get_robot_state     -> {x, y, yaw, v_linear, v_angular, mode, fault, ...}
    POST /set_velocity        -> {accepted, linear, angular}
    POST /drive_forward       -> {accepted, distance, eta_s}
    POST /turn                -> {accepted, angle_rad, eta_s}
    POST /stop_robot          -> {halted_at}
    POST /reset_to_home       -> teleport to start pose
    POST /prompt              -> natural language

Robot window: same omnilink_chat plugin as the arm bridge. Wire protocol
is identical: "prompt:<text>", "stop", "configure" inbound;
"configure:<json>" / "status:..." / "agent:..." / "tool:..." outbound.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

from omnisim import Supervisor

import os as _os
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
if _THIS_DIR not in _sys.path:
    _sys.path.insert(0, _THIS_DIR)
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

from _mobile_configs import MOBILE_CONFIGS, WHEEL_MOTORS, get_config  # noqa: E402

try:
    from _omnilink_relay import OmniLinkRelay, Tool, is_enabled as omnilink_enabled, get_omni_key  # noqa: E402
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


# ── CLI ──────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="husky")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--name", default=None,
                   help=("Override the agent id used in the OmniLink "
                         "profile name and Axis robot_id. Defaults to "
                         "--robot. Use when multiple bases of the same "
                         "kind share a world (e.g. 4 Huskies in a swarm)."))
    args, _ = p.parse_known_args()
    return args


# ── Helpers ──────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def yaw_from_orientation(o) -> float:
    # Webots orientation is a 9-element row-major rotation matrix.
    # Z-up convention: yaw = atan2(o[3], o[0]).
    return math.atan2(o[3], o[0])


# ── Bridge ───────────────────────────────────────────────────────────

class MobileBridge:
    """Owns wheel motors and pose for one URDF mobile base."""

    def __init__(self, robot: Supervisor, cfg: dict, robot_id: str) -> None:
        self.robot = robot
        self.cfg = cfg
        self.robot_id = robot_id
        self.timestep = int(robot.getBasicTimeStep())

        layout = cfg["layout"]
        if layout not in WHEEL_MOTORS:
            raise ValueError(f"Unknown wheel layout {layout!r}")
        wheel_names = WHEEL_MOTORS[layout]
        self.left_motors = [robot.getDevice(n) for n in wheel_names["left"]]
        self.right_motors = [robot.getDevice(n) for n in wheel_names["right"]]
        missing = [n for n, m in zip(wheel_names["left"] + wheel_names["right"],
                                     self.left_motors + self.right_motors) if m is None]
        if missing:
            print(f"[omnilink_mobile_bridge] WARNING: missing motors: {missing}")
        for m in self.left_motors + self.right_motors:
            if m is not None:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)

        # Optional init poses -- tuck arms, lower torso, etc. for
        # robots whose default URDF spawn pose looks like a failure
        # (mobile-manipulator robots with extended arms, etc.).
        for joint_name, q in (cfg.get("init_poses") or {}).items():
            m = robot.getDevice(joint_name + "_motor") or robot.getDevice(joint_name)
            if m is not None:
                try:
                    m.setPosition(q)
                except Exception as e:
                    print(f"[omnilink_mobile_bridge] init_pose {joint_name}={q} failed: {e}")

        self.r = cfg["wheel_radius_m"]
        self.ht = cfg["half_track_m"]
        self.v_max = cfg["max_wheel_speed_radps"]
        self.v_max_linear = self.v_max * self.r
        self.v_max_angular = self.v_max * self.r / self.ht

        self.cruise_linear = cfg["cruise_frac"] * self.v_max_linear
        self.spin_speed = cfg["spin_speed"]

        # Pose tracking.
        self.self_node = robot.getSelf()
        self.start_xyz = list(self.self_node.getPosition())
        self.start_yaw = yaw_from_orientation(self.self_node.getOrientation())
        self.last_xy = (self.start_xyz[0], self.start_xyz[1])
        self.last_yaw = self.start_yaw
        self.v_linear = 0.0
        self.v_angular = 0.0

        # Motion state machine.
        self.lock = threading.RLock()
        # ("idle", {}) | ("velocity", {"l", "a"}) | ("drive", {...}) | ("turn", {...})
        self.motion = ("idle", {})
        self.fault: Optional[str] = None
        self.last_tick_at = time.time()

        # wwi outbox.
        self.window_outbox: List[str] = []
        self.window_configured = False

        self.capabilities = {
            "layout": layout,
            "wheel_radius_m": self.r,
            "half_track_m": self.ht,
            "max_linear_m_s": self.v_max_linear,
            "max_angular_rad_s": self.v_max_angular,
            "cruise_linear_m_s": self.cruise_linear,
        }

    # ── Pose / velocity readout ───────────────────────────────────

    def _read_pose(self) -> Tuple[float, float, float]:
        p = self.self_node.getPosition()
        yaw = yaw_from_orientation(self.self_node.getOrientation())
        return p[0], p[1], yaw

    # ── Wheel commanding ──────────────────────────────────────────

    def _command_velocity(self, linear: float, angular: float) -> Tuple[float, float]:
        """Convert (linear m/s, angular rad/s) to (left rad/s, right rad/s),
        apply, return clamped values."""
        linear = clamp(linear, -self.v_max_linear, self.v_max_linear)
        angular = clamp(angular, -self.v_max_angular, self.v_max_angular)
        # Skid-steer / diff-drive kinematics.
        v_left = (linear - angular * self.ht) / self.r
        v_right = (linear + angular * self.ht) / self.r
        max_abs = max(abs(v_left), abs(v_right), 1e-9)
        if max_abs > self.v_max:
            scale = self.v_max / max_abs
            v_left *= scale
            v_right *= scale
        for m in self.left_motors:
            if m is not None:
                m.setVelocity(v_left)
        for m in self.right_motors:
            if m is not None:
                m.setVelocity(v_right)
        return v_left, v_right

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    # ── Tick loop ─────────────────────────────────────────────────

    def tick(self, dt_s: float) -> None:
        x, y, yaw = self._read_pose()
        dx = x - self.last_xy[0]
        dy = y - self.last_xy[1]
        if dt_s > 1e-4:
            self.v_linear = math.cos(yaw) * dx / dt_s + math.sin(yaw) * dy / dt_s
            self.v_angular = wrap_pi(yaw - self.last_yaw) / dt_s
        self.last_xy = (x, y)
        self.last_yaw = yaw
        self.last_tick_at = time.time()

        with self.lock:
            kind, p = self.motion
        if kind == "idle":
            self._command_velocity(0.0, 0.0)
        elif kind == "velocity":
            self._command_velocity(p["l"], p["a"])
        elif kind == "drive":
            # Drive forward / backward a target distance.
            travelled = math.sqrt((x - p["x0"]) ** 2 + (y - p["y0"]) ** 2)
            if travelled >= p["distance"] or (time.time() - p["t0"]) > p["timeout_s"]:
                with self.lock:
                    self.motion = ("idle", {})
                self._command_velocity(0.0, 0.0)
                self.queue_window(f"system:drive complete (travelled {travelled:.2f} m)")
            else:
                self._command_velocity(p["speed"], 0.0)
        elif kind == "turn":
            err = wrap_pi(p["target_yaw"] - yaw)
            if abs(err) < 0.04 or (time.time() - p["t0"]) > p["timeout_s"]:
                with self.lock:
                    self.motion = ("idle", {})
                self._command_velocity(0.0, 0.0)
                self.queue_window(f"system:turn complete (yaw err {err:+.3f} rad)")
            else:
                w = clamp(err * 3.0, -self.spin_speed, self.spin_speed)
                # Make sure we don't pivot too slowly near zero error.
                if 0 < abs(w) < 0.4:
                    w = 0.4 * (1 if w > 0 else -1)
                self._command_velocity(0.0, w)

    # ── Actions (HTTP + intent share these) ───────────────────────

    def act_stop(self) -> dict:
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        return {"halted_at": time.time()}

    def act_set_velocity(self, linear: float, angular: float) -> dict:
        with self.lock:
            self.motion = ("velocity", {"l": linear, "a": angular})
        return {"accepted": True, "linear": linear, "angular": angular}

    def act_drive_forward(self, distance: float, speed: Optional[float] = None) -> dict:
        x, y, _ = self._read_pose()
        actual_speed = speed if speed is not None else self.cruise_linear
        signed_speed = actual_speed if distance >= 0 else -actual_speed
        target = abs(distance)
        eta = target / max(abs(actual_speed), 1e-6)
        with self.lock:
            self.motion = ("drive", {
                "x0": x, "y0": y,
                "distance": target,
                "speed": signed_speed,
                "t0": time.time(),
                "timeout_s": eta * 3.0 + 2.0,
            })
        return {"accepted": True, "distance": distance, "eta_s": eta}

    def act_turn(self, angle_rad: float) -> dict:
        _, _, yaw = self._read_pose()
        target = wrap_pi(yaw + angle_rad)
        eta = abs(angle_rad) / max(self.spin_speed, 1e-6)
        with self.lock:
            self.motion = ("turn", {
                "target_yaw": target,
                "t0": time.time(),
                "timeout_s": eta * 3.0 + 2.0,
            })
        return {"accepted": True, "angle_rad": angle_rad, "eta_s": eta}

    def act_reset_to_home(self) -> dict:
        try:
            self.self_node.getField("translation").setSFVec3f(list(self.start_xyz))
            # Reset orientation to identity-yaw.
            self.self_node.getField("rotation").setSFRotation([0, 0, 1, self.start_yaw])
            self.self_node.resetPhysics()
        except Exception as e:
            return {"error": f"reset_failed: {e}"}
        with self.lock:
            self.motion = ("idle", {})
        self._command_velocity(0.0, 0.0)
        return {"accepted": True, "start_xyz": self.start_xyz}

    def get_state(self) -> dict:
        x, y, yaw = self._read_pose()
        with self.lock:
            kind = self.motion[0]
        return {
            "id": self.robot_id,
            "model": self.cfg["model"],
            "x": x, "y": y, "yaw": yaw,
            "v_linear": self.v_linear,
            "v_angular": self.v_angular,
            "mode": kind,
            "fault": self.fault,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
        }


# ── Intent router ────────────────────────────────────────────────────

class IntentRouter:
    NUMBER = r"(-?\d+\.?\d*)"

    def __init__(self, bridge: MobileBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        if re.search(r"\b(stop|halt|freeze|brake)\b", s):
            self.bridge.act_stop()
            return {
                "agent": "Stopping wheels.",
                "tools": [("stop_robot", "ok", "v=0")],
            }

        if re.search(r"\b(reset|home|teleport.*start)\b", s):
            res = self.bridge.act_reset_to_home()
            ok = "error" not in res
            return {
                "agent": "Teleporting back to start." if ok else res["error"],
                "tools": [("reset_to_home", "ok" if ok else "err",
                          f"start=({res.get('start_xyz', ['?', '?'])[0]:.2f}, {res.get('start_xyz', ['?', '?'])[1]:.2f})" if ok else res["error"])],
            }

        # Spin in place.
        if re.search(r"\b(spin|rotate)\b", s) and not re.search(r"degree|rad|left|right", s):
            self.bridge.act_set_velocity(0.0, self.bridge.spin_speed)
            return {
                "agent": "Spinning in place. Send 'stop' to halt.",
                "tools": [("set_velocity", "ok", f"a={self.bridge.spin_speed:+.2f} rad/s")],
            }

        if re.search(r"\b(circle|loop)\b", s):
            self.bridge.act_set_velocity(self.bridge.cruise_linear * 0.6, self.bridge.spin_speed * 0.6)
            return {
                "agent": "Driving in a circle. Send 'stop' to halt.",
                "tools": [("set_velocity", "ok", "circle")],
            }

        # Turn N degrees / radians, with optional direction.
        m = re.search(r"turn(?:\s+(left|right))?\s+(?:about\s+)?" + self.NUMBER + r"\s*(deg|degree|rad|radian)?", s)
        if m:
            direction = m.group(1)
            val = float(m.group(2))
            unit = (m.group(3) or "deg").lower()
            angle = val if unit.startswith("rad") else math.radians(val)
            if direction == "right":
                angle = -abs(angle)
            elif direction == "left":
                angle = abs(angle)
            res = self.bridge.act_turn(angle)
            return {
                "agent": f"Turning {math.degrees(angle):+.0f}° (~{res['eta_s']:.1f}s).",
                "tools": [("turn", "ok", f"angle={math.degrees(angle):+.0f}°")],
            }
        if re.search(r"\bturn (left|right)\b", s):
            direction = re.search(r"\bturn (left|right)\b", s).group(1)
            sign = 1 if direction == "left" else -1
            res = self.bridge.act_turn(sign * math.pi / 2)
            return {
                "agent": f"Turning {direction} 90°.",
                "tools": [("turn", "ok", f"angle={sign*90}°")],
            }
        if re.search(r"\b(turn around|u[- ]?turn|about face)\b", s):
            res = self.bridge.act_turn(math.pi)
            return {
                "agent": "Turning around (180°).",
                "tools": [("turn", "ok", "angle=180°")],
            }

        # Drive forward / back N (meters|m|cm).
        m = re.search(r"\b(forward|forwards|ahead|back|backward|backwards|reverse)\b[^-\d]*" + self.NUMBER + r"\s*(m|meter|metre|metres|meters|cm|centi)?", s)
        if m:
            direction = m.group(1)
            val = float(m.group(2))
            unit = (m.group(3) or "m").lower()
            d = val / 100.0 if unit.startswith("cm") or unit.startswith("centi") else val
            if direction.startswith("back") or direction == "reverse":
                d = -d
            res = self.bridge.act_drive_forward(d)
            return {
                "agent": f"Driving {direction} {abs(d):.2f} m (~{res['eta_s']:.1f}s).",
                "tools": [("drive_forward", "ok", f"distance={d:+.2f} m")],
            }
        m = re.search(r"\b(forward|forwards|ahead)\b", s)
        if m:
            res = self.bridge.act_drive_forward(1.0)
            return {
                "agent": "Driving forward 1 m by default.",
                "tools": [("drive_forward", "ok", "1.00 m")],
            }
        m = re.search(r"\b(back|reverse)\b", s)
        if m:
            res = self.bridge.act_drive_forward(-1.0)
            return {
                "agent": "Backing up 1 m by default.",
                "tools": [("drive_forward", "ok", "-1.00 m")],
            }

        # Set velocity directly: "set velocity 0.3 0.0"
        m = re.search(r"(?:set\s+)?velocity\s+" + self.NUMBER + r"[,\s]+" + self.NUMBER, s)
        if m:
            lin = float(m.group(1))
            ang = float(m.group(2))
            self.bridge.act_set_velocity(lin, ang)
            return {
                "agent": f"Setting velocity to ({lin:.2f} m/s, {ang:.2f} rad/s).",
                "tools": [("set_velocity", "ok", f"({lin:+.2f}, {ang:+.2f})")],
            }

        # State query.
        if re.search(r"\b(status|state|where|pose|telemetry|odometry)\b", s):
            st = self.bridge.get_state()
            return {
                "agent": f"x={st['x']:+.2f}, y={st['y']:+.2f}, yaw={math.degrees(st['yaw']):+.0f}°, v={st['v_linear']:+.2f} m/s, mode={st['mode']}.",
                "tools": [("get_robot_state", "ok", st['mode'])],
            }

        return {
            "agent": ("I don't recognise that. Try: "
                      "\"forward 1 m\", \"back 50 cm\", "
                      "\"turn left 90 degrees\", \"turn around\", "
                      "\"spin\", \"stop\", \"reset\"."),
            "tools": [],
        }


# ── HTTP ─────────────────────────────────────────────────────────────

def make_handler(bridge: MobileBridge, router: IntentRouter, relay: Any = None):
    class _H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _json(self, code, obj):
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except Exception:
                return {}

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/usage":
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "latest": relay.latest_usage(),
                })
            return self._json(404, {"error": "not_found"})

        def do_POST(self):
            body = self._read_json()
            p = self.path.rstrip("/")
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.cfg["model"],
                    "capabilities": bridge.capabilities,
                }])
            if p == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if p == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if p == "/set_velocity":
                return self._json(200, bridge.act_set_velocity(
                    float(body.get("linear", 0.0)), float(body.get("angular", 0.0))))
            if p == "/drive_forward":
                return self._json(200, bridge.act_drive_forward(
                    float(body.get("distance", 1.0)),
                    body.get("speed")))
            if p == "/turn":
                return self._json(200, bridge.act_turn(float(body.get("angle", 0.0))))
            if p == "/prompt":
                text = (body.get("text") or "").strip()
                if not text:
                    return self._json(400, {"error": "text required"})
                if relay is not None:
                    return self._json(200, relay.dispatch_sync(text))
                result = router.dispatch(text)
                return self._json(200, {
                    "response": result["agent"],
                    "actions": [{"tool": t[0], "result": t[1], "summary": t[2]}
                                for t in result["tools"]],
                })
            if p == "/tool":
                # Platform-side tool callback. omnilink-agents.com web UI
                # POSTs {"tool": "<name>", ...args} after producing tool
                # calls on its side; we dispatch the same registered Tool
                # the relay loop dispatches and return its result.
                tool_name = (body.pop("tool", None) or "").strip()
                if not tool_name:
                    return self._json(400, {"error": "tool name required"})
                if relay is None or tool_name not in getattr(relay, "tools", {}):
                    return self._json(503, {
                        "status": "err",
                        "tool": tool_name,
                        "error": "tool_not_registered",
                    })
                try:
                    result = relay.tools[tool_name].dispatch(body)
                    return self._json(200, {
                        "status": "ok",
                        "tool": tool_name,
                        "result": result,
                    })
                except Exception as e:
                    return self._json(500, {
                        "status": "err",
                        "tool": tool_name,
                        "error": repr(e),
                    })
            return self._json(404, {"error": "not_found", "path": p})
    return _H


def start_http(bridge: MobileBridge, router: IntentRouter, port: int, relay: Any = None) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, router, relay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[omnilink_mobile_bridge] HTTP on http://127.0.0.1:{port}")
    return server


# ── OmniLink tool builders ───────────────────────────────────────────

def build_mobile_tools(bridge: MobileBridge) -> List[Any]:
    if Tool is None:
        return []
    return [
        Tool(
            name="drive_forward",
            description=(
                "Drive a specified distance along the current heading. "
                "Positive distance is forward, negative is reverse. Speed "
                "is optional (defaults to cruise speed)."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "distance": {"type": "number", "description": "Distance in metres. Positive=forward."},
                    "speed": {"type": "number", "description": "Optional speed override (m/s). Omit to use cruise."},
                },
                "required": ["distance"],
            },
            dispatch=lambda args: bridge.act_drive_forward(
                float(args.get("distance", 0.0)), args.get("speed")),
        ),
        Tool(
            name="turn",
            description="Turn in place by a signed angle. Positive=counter-clockwise (left).",
            parameters={
                "type": "object",
                "properties": {
                    "angle_rad": {"type": "number", "description": "Turn angle in radians. Positive=left."},
                },
                "required": ["angle_rad"],
            },
            dispatch=lambda args: bridge.act_turn(float(args.get("angle_rad", 0.0))),
        ),
        Tool(
            name="set_velocity",
            description=(
                "Set continuous (linear, angular) velocity. Robot keeps moving until "
                "another command lands. Use stop_robot to halt."
            ),
            parameters={
                "type": "object",
                "properties": {
                    "linear": {"type": "number", "description": "Forward velocity (m/s)."},
                    "angular": {"type": "number", "description": "Yaw rate (rad/s, +=left)."},
                },
                "required": ["linear", "angular"],
            },
            dispatch=lambda args: bridge.act_set_velocity(
                float(args.get("linear", 0.0)), float(args.get("angular", 0.0))),
        ),
        Tool(
            name="stop_robot",
            description="Emergency halt — zero both wheel velocities.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_stop(),
        ),
        Tool(
            name="reset_to_home",
            description="Teleport the robot back to its spawn pose (supervisor reset).",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_reset_to_home(),
        ),
        Tool(
            name="get_robot_state",
            description="Read pose (x, y, yaw), measured velocity, and current mode.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.get_state(),
        ),
    ]


def build_mobile_main_task(bridge: MobileBridge) -> str:
    return (
        f"You drive a {bridge.cfg['model']} mobile base in OmniSim through the "
        f"OmniLink-OmniSim bridge. Max speed ~{bridge.v_max_linear:.2f} m/s, "
        f"max yaw rate ~{bridge.v_max_angular:.2f} rad/s.\n\n"
        "Rules:\n"
        "- Translate the operator's request into ONE tool call when motion is implied.\n"
        "- For 'forward N m' / 'back N m' use drive_forward(distance=N or -N).\n"
        "- For 'turn left/right N degrees' use turn(angle_rad=±N*pi/180).\n"
        "- For 'spin' or 'circle' use set_velocity (the base keeps moving).\n"
        "- 'stop'/'halt' -> stop_robot. 'reset' -> reset_to_home.\n"
        "- Keep the final text response short -- one sentence."
    )


def setup_omnilink_relay(bridge: MobileBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{bridge.robot_id}"
        tools = build_mobile_tools(bridge)
        main_task = build_mobile_main_task(bridge)
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Push a per-robot profile so operators can pick this base in
        # the omnilink-agents.com web UI and chat to the sim from
        # there. Tool calls round-trip back via /tool below.
        from _omnilink_relay import profile_sync
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(
                client=relay._client,
                agent_name=agent_name,
                main_task=main_task,
                tool_defs=[t.to_definition() for t in tools],
                engine=relay.engine,
                tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
            )
        print(f"[omnilink_mobile_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        print(f"[omnilink_mobile_bridge] OmniLink relay setup failed: {e}")
        return None


# ── wwi plumbing ─────────────────────────────────────────────────────

def push_configure(bridge: MobileBridge, relay: Any) -> None:
    agent_label = (
        f"OmniLink relay ({_os.environ.get('OMNILINK_ENGINE', 'g4-engine')})"
        if relay is not None else "local intent (regex)"
    )
    cfg = {
        "robot": bridge.cfg["model"],
        "robot_class": "mobile base",
        "agent": agent_label,
        "suggestions": [
            "forward 1 m",
            "turn left 90 degrees",
            "back 50 cm",
            "spin",
            "stop",
        ],
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def _on_relay_event(bridge: MobileBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        bridge.queue_window(
            f"tool:{payload.get('name', '?')}:{payload.get('status', 'ok')}:{payload.get('summary', '')}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def handle_wwi_message(bridge: MobileBridge, router: IntentRouter, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay)
        return
    if msg.startswith("stop"):
        bridge.act_stop()
        bridge.queue_window("agent:Stop received.")
        bridge.queue_window("tool:stop_robot:ok:halted")
        bridge.queue_window("status:idle")
        return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        if relay is not None:
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))
            return
        bridge.queue_window("status:thinking")
        result = router.dispatch(text)
        for (tool, status, summary) in result["tools"]:
            bridge.queue_window(f"tool:{tool}:{status}:{summary}")
        bridge.queue_window("agent:" + result["agent"])
        bridge.queue_window("status:idle")
        return
    if msg.startswith("audio_in:"):
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)")
            return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}")
            return
        bridge.queue_window("status:transcribing")

        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle")
                return
            bridge.queue_window("transcript:" + text)
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return
    bridge.queue_window("system:Unknown window message: " + msg[:200])


# ── Main ─────────────────────────────────────────────────────────────

def main() -> int:
    args = _parse_args()
    cfg = get_config(args.robot)
    robot = Supervisor()
    # --name overrides the agent id so multiple bases of the same kind
    # (e.g. 4 Huskies in a swarm) get distinct OmniLink profiles + Axis ids.
    robot_id = args.name or args.robot
    bridge = MobileBridge(robot, cfg, robot_id)
    router = IntentRouter(bridge)
    relay = setup_omnilink_relay(bridge, http_port=args.port)
    start_http(bridge, router, args.port, relay)
    print(f"[omnilink_mobile_bridge] {cfg['model']} ready as '{robot_id}' "
          f"layout={cfg['layout']} r={bridge.r:.3f} ht={bridge.ht:.3f} "
          f"({'OmniLink' if relay else 'local'})")

    timestep = bridge.timestep
    dt_s = timestep / 1000.0
    while robot.step(timestep) != -1:
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception as e:
                print(f"[omnilink_mobile_bridge] wwiSendText failed: {e}")
        bridge.tick(dt_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
