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

"""BridgeBase -- abstract Webots-less bridge for real robots.

This is the sim-to-real seam: the OmniLink agent code (relay, intent
router, tool definitions) is byte-identical between OmniSim and a real
robot integration. The only thing that changes is what `act_*` does:
in OmniSim it calls `motor.setPosition(...)`; on a real robot it calls
into your fleet's SDK.

The stub here provides:

  - `BridgeBase` -- abstract class with the methods every robot
    must implement (`act_stop`, `act_set_velocity`, `act_drive_forward`,
    `act_turn`, `act_set_joint_positions`, `act_set_tcp_target`,
    `act_open_gripper`, `act_close_gripper`, `act_reset_to_home`,
    `get_state`). Subclasses pick the subset that applies to their
    robot kind (arm / mobile / both).

  - `serve_http(bridge, port)` -- exactly the HTTP surface the OmniSim
    arm + mobile bridges expose (`/list_robots`, `/get_robot_state`,
    `/prompt`, `/tool`, `/usage`, ...), so an OmniLink-Foreman or
    OmniLink-Picker driving the simulated robot today drives the real
    robot tomorrow by changing one URL.

To build your own real-robot bridge, copy `arm_bridge_stub.py` (next
to this file), replace `MockArmDriver` with your robot SDK's client,
run it. That's the whole sim-to-real story.

This file has zero Webots imports and zero OmniSim-internal imports.
It depends only on the Python stdlib + optionally the omnilink package
for the OmniLink relay path (which the bridge can run in or skip).
"""

from __future__ import annotations

import abc
import inspect
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional

from .http_security import (
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    finite_number,
    number_list,
    read_json,
    require_field,
    validate_bind,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)


class BridgeBase(abc.ABC):
    """Subclass and implement the act_* methods that apply to your
    robot. Methods you don't implement default to a clean "unsupported"
    response that the OmniLink relay surfaces as a tool error -- no
    crashes, no NotImplementedError leaking up the wire.
    """

    # Filled in by the subclass.
    robot_id: str = "robot_0"
    model: str = "GenericRobot"
    capabilities: Dict[str, Any] = {}

    # ── Stop is mandatory. Every other action is opt-in. ──────────

    @abc.abstractmethod
    def act_stop(self) -> Dict[str, Any]:
        """Emergency halt. Idempotent. ALWAYS available -- this is the
        one method every bridge must implement. Return at minimum
        {"halted_at": <unix-seconds>}."""
        raise NotImplementedError

    # ── State read ────────────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        """Return a snapshot of the robot's state. Subclasses should
        override; the base returns a minimal record so /get_robot_state
        always answers something."""
        return {
            "id": self.robot_id,
            "model": self.model,
            "last_tick_at": time.time(),
        }

    # ── Optional motion actions. Default to "not supported". ──────

    def act_set_velocity(self, linear: float, angular: float) -> Dict[str, Any]:
        return {"error": "set_velocity not supported by this robot kind"}

    def act_drive_forward(self, distance: float, speed: Optional[float] = None) -> Dict[str, Any]:
        return {"error": "drive_forward not supported by this robot kind"}

    def act_turn(self, angle_rad: float) -> Dict[str, Any]:
        return {"error": "turn not supported by this robot kind"}

    def act_set_joint_positions(self, q: List[float], duration_s: float = 1.2) -> Dict[str, Any]:
        return {"error": "set_joint_positions not supported by this robot kind"}

    def act_set_tcp_target(self, xyz: List[float]) -> Dict[str, Any]:
        return {"error": "set_tcp_target not supported by this robot kind"}

    def act_reset_to_home(self) -> Dict[str, Any]:
        return {"error": "reset_to_home not supported by this robot kind"}

    def act_open_gripper(self) -> Dict[str, Any]:
        return {"error": "open_gripper not supported (no gripper on this robot)"}

    def act_close_gripper(self) -> Dict[str, Any]:
        return {"error": "close_gripper not supported (no gripper on this robot)"}

    def act_set_gripper_width(self, width: float) -> Dict[str, Any]:
        return {"error": "set_gripper_width not supported (no gripper on this robot)"}

    def act_grasp(self, force: Optional[float] = None,
                  width: Optional[float] = None) -> Dict[str, Any]:
        return {"error": "grasp not supported (no gripper on this robot)"}

    def act_release(self) -> Dict[str, Any]:
        return {"error": "release not supported (no gripper on this robot)"}

    # ── Natural-language prompt fallback ──────────────────────────

    def act_prompt(self, text: str) -> Dict[str, Any]:
        """If you've wired this bridge into an OmniLink relay, plug
        the relay in here. Default fallback echoes the prompt so /prompt
        always returns something useful even with no LLM."""
        return {
            "response": f"(no agent attached) received: {text}",
            "actions": [],
        }


# ── HTTP server (Axis-normalised, mirrors omnilink_*_bridge) ────────

def _make_handler(
    bridge: BridgeBase,
    *,
    token: str = "",
    trusted_origins: Optional[set[str]] = None,
) -> Any:
    """Returns a BaseHTTPRequestHandler subclass bound to `bridge`. The
    routes match the OmniSim bridges' surface exactly so any existing
    OmniLink agent (Foreman, Picker, Roomba, Axis) drives this bridge
    by pointing its callback URL at us."""

    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted = trusted_origins or allowed_origins()

    class _H(BaseHTTPRequestHandler):
        def log_message(self, *args, **kwargs):
            return

        def _json(self, code: int, obj: Any, origin: Optional[str] = None) -> None:
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _guard(self, *, authorize: bool = False) -> Optional[str]:
            check_protocol_version(self.headers)
            origin = checked_origin(self.headers, trusted)
            if authorize:
                check_authorization(self.headers, token)
            return origin

        def _failure(self, exc: RequestError, origin: Optional[str] = None) -> None:
            self._json(exc.status, error_envelope(exc.code, exc.message, exc.details), origin)

        def _invoke(self, fn: Callable[[], Any], *, stop: bool = False) -> Any:
            request_ids.claim(
                getattr(self, "_request_path", self.path),
                getattr(self, "_request_id", None),
            )
            if stop:
                return fn()
            with action_lock:
                return fn()

        def _read_json(self, *, allow_empty: bool = True) -> Dict[str, Any]:
            return read_json(self, allow_empty=allow_empty)

        def _cors_preflight(self) -> Optional[str]:
            origin = self._guard(authorize=False)
            requested_headers = (self.headers.get("Access-Control-Request-Headers") or "").lower()
            allowed = {"content-type", "authorization", "x-omnisim-token"}
            requested = {h.strip() for h in requested_headers.split(",") if h.strip()}
            if not requested.issubset(allowed):
                raise RequestError(403, "header_not_allowed", "CORS request includes a disallowed header.")
            return origin

        def do_OPTIONS(self) -> None:
            try:
                origin = self._cors_preflight()
                self.send_response(204)
                if origin:
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )
                self.end_headers()
            except RequestError as exc:
                self._failure(exc)

        def do_GET(self) -> None:
            origin: Optional[str] = None
            try:
                origin = self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path in ("/state", "/get_robot_state"):
                    return self._json(200, bridge.get_state(), origin)
                if path in ("/capabilities", "/list_robots"):
                    return self._json(200, [{
                        "id": bridge.robot_id, "model": bridge.model,
                        "capabilities": bridge.capabilities,
                    }], origin)
                if path == "/healthz":
                    return self._json(200, {"ok": True}, origin)
                if path == "/protocol":
                    return self._json(200, {
                        "ok": True,
                        "omnisim_wire": WIRE_VERSION,
                        "service": WIRE_SERVICE,
                        "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                        "instance": {"name": bridge.__class__.__name__, "robot_id": bridge.robot_id},
                        "extensions": [],
                    }, origin)
                if path == "/usage":
                    return self._json(200, {"enabled": False}, origin)
                return self._json(404, error_envelope("not_found", "Endpoint not found."), origin)
            except RequestError as exc:
                self._failure(exc, origin)

        def do_POST(self) -> None:
            origin: Optional[str] = None
            try:
                origin = self._guard(authorize=True)
                body = self._read_json()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                self._request_path = path
                self._request_id = validate_request_id(body.get("id"))
                if path in ("/state", "/get_robot_state"):
                    return self._json(200, bridge.get_state(), origin)
                if path in ("/list_robots", "/capabilities"):
                    return self._json(200, [{
                        "id": bridge.robot_id, "model": bridge.model,
                        "capabilities": bridge.capabilities,
                    }], origin)
                if path == "/stop_robot":
                    return self._json(200, self._invoke(bridge.act_stop, stop=True), origin)
                if path == "/reset_to_home":
                    result = self._invoke(bridge.act_reset_to_home)
                    return self._action_response(result, origin)
                if path == "/set_velocity":
                    linear = finite_number(require_field(body, "linear"), "linear")
                    angular = finite_number(require_field(body, "angular"), "angular")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_velocity(linear, angular)), origin
                    )
                if path == "/drive_forward":
                    distance = finite_number(require_field(body, "distance"), "distance")
                    speed = body.get("speed")
                    if speed is not None:
                        speed = finite_number(speed, "speed")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_drive_forward(distance, speed)), origin
                    )
                if path == "/turn":
                    angle_value = body.get("angle_rad", body.get("angle"))
                    if angle_value is None:
                        raise RequestError(400, "missing_field", "Required field 'angle_rad' is missing.")
                    angle = finite_number(angle_value, "angle_rad")
                    return self._action_response(self._invoke(lambda: bridge.act_turn(angle)), origin)
                if path == "/set_joint_positions":
                    q = number_list(require_field(body, "q"), "q")
                    duration = body.get("duration_s", 1.2)
                    duration_s = finite_number(duration, "duration_s")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_joint_positions(q, duration_s)), origin
                    )
                if path == "/set_tcp_target":
                    xyz = number_list(require_field(body, "xyz"), "xyz", length=3)
                    return self._action_response(self._invoke(lambda: bridge.act_set_tcp_target(xyz)), origin)
                if path == "/open_gripper":
                    return self._action_response(self._invoke(bridge.act_open_gripper), origin)
                if path == "/close_gripper":
                    return self._action_response(self._invoke(bridge.act_close_gripper), origin)
                if path == "/set_gripper_width":
                    width = finite_number(require_field(body, "width"), "width")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_set_gripper_width(width)), origin
                    )
                if path == "/grasp":
                    force = body.get("force")
                    width = body.get("width")
                    if force is not None:
                        force = finite_number(force, "force")
                    if width is not None:
                        width = finite_number(width, "width")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_grasp(force=force, width=width)), origin
                    )
                if path == "/release":
                    return self._action_response(self._invoke(bridge.act_release), origin)
                if path == "/prompt":
                    text_value = require_field(body, "text")
                    if not isinstance(text_value, str) or not text_value.strip():
                        raise RequestError(400, "invalid_type", "Field 'text' must be a non-empty string.")
                    return self._action_response(
                        self._invoke(lambda: bridge.act_prompt(text_value.strip())), origin
                    )
                if path == "/tool":
                    tool_value = require_field(body, "tool")
                    if not isinstance(tool_value, str) or not tool_value.strip():
                        raise RequestError(400, "invalid_type", "Field 'tool' must be a non-empty string.")
                    tool_name = tool_value.strip()
                    args = {k: v for k, v in body.items() if k not in ("tool", "id")}
                    result = self._invoke(lambda: _dispatch_tool(bridge, tool_name, args), stop=tool_name == "stop_robot")
                    status = "err" if isinstance(result, dict) and result.get("error") else "ok"
                    code = 404 if status == "err" and str(result.get("error", "")).startswith("unknown tool") else 200
                    return self._json(code, {"status": status, "tool": tool_name, "result": result}, origin)
                return self._json(
                    404, error_envelope("not_found", "Endpoint not found.", {"path": path}), origin
                )
            except RequestError as exc:
                self._failure(exc, origin)
            except (TypeError, ValueError) as exc:
                self._failure(RequestError(400, "invalid_arguments", str(exc)), origin)
            except Exception as exc:
                self._json(
                    500,
                    error_envelope("internal_error", "Robot action failed.", {"type": type(exc).__name__}),
                    origin,
                )

        def _action_response(self, result: Any, origin: Optional[str]) -> None:
            if isinstance(result, dict) and result.get("error"):
                self._json(501, error_envelope("not_supported", str(result["error"])), origin)
                return
            self._json(200, result, origin)

    return _H


_TOOL_ALIASES: Dict[str, str] = {
    "stop_robot":          "act_stop",
    "get_robot_state":     "get_state",
    "reset_to_home":       "act_reset_to_home",
    "set_velocity":        "act_set_velocity",
    "drive_forward":       "act_drive_forward",
    "turn":                "act_turn",
    "set_joint_positions": "act_set_joint_positions",
    "set_tcp_target":      "act_set_tcp_target",
    "open_gripper":        "act_open_gripper",
    "close_gripper":       "act_close_gripper",
    "set_gripper_width":   "act_set_gripper_width",
    "grasp":               "act_grasp",
    "release":             "act_release",
}


def _dispatch_tool(bridge: BridgeBase, tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """Map an OmniLink tool name onto a bridge act_* method."""
    method_name = _TOOL_ALIASES.get(tool_name)
    if method_name is None:
        return {"error": f"unknown tool: {tool_name}", "known": sorted(_TOOL_ALIASES.keys())}
    method = getattr(bridge, method_name, None)
    if method is None:
        return {"error": f"bridge does not implement {method_name}"}
    try:
        inspect.signature(method).bind(**args)
    except TypeError as exc:
        raise RequestError(400, "invalid_arguments", str(exc), {"tool": tool_name}) from exc
    return method(**args)


def serve_http(
    bridge: BridgeBase,
    port: int = 8765,
    *,
    host: str = "127.0.0.1",
    token: Optional[str] = None,
    origins: Optional[List[str]] = None,
) -> ThreadingHTTPServer:
    """Spin up the bridge HTTP server in a background thread. Returns
    the server instance so the caller can `server.shutdown()` cleanly."""
    resolved_token = configured_token(token)
    validate_bind(host, resolved_token)
    server = ThreadingHTTPServer(
        (host, port),
        _make_handler(bridge, token=resolved_token, trusted_origins=allowed_origins(origins)),
    )
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[bridge_base] HTTP listening on http://{host}:{server.server_address[1]}")
    return server
