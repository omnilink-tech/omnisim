# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

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
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional


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

def _make_handler(bridge: BridgeBase) -> Any:
    """Returns a BaseHTTPRequestHandler subclass bound to `bridge`. The
    routes match the OmniSim bridges' surface exactly so any existing
    OmniLink agent (Foreman, Picker, Roomba, Axis) drives this bridge
    by pointing its callback URL at us."""

    class _H(BaseHTTPRequestHandler):
        def log_message(self, *args, **kwargs):
            return

        def _json(self, code: int, obj: Any) -> None:
            data = json.dumps(obj, default=str).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self) -> Dict[str, Any]:
            n = int(self.headers.get("Content-Length", "0"))
            if n <= 0:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode("utf-8") or "{}")
            except Exception:
                return {}

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/usage":
                return self._json(200, {"enabled": False})
            return self._json(404, {"error": "not_found"})

        def do_POST(self) -> None:
            body = self._read_json()
            path = self.path.rstrip("/")
            if path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if path in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if path == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if path == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if path == "/set_velocity":
                return self._json(200, bridge.act_set_velocity(
                    float(body.get("linear", 0.0)), float(body.get("angular", 0.0))))
            if path == "/drive_forward":
                return self._json(200, bridge.act_drive_forward(
                    float(body.get("distance", 1.0)),
                    body.get("speed")))
            if path == "/turn":
                return self._json(200, bridge.act_turn(float(body.get("angle_rad", body.get("angle", 0.0)))))
            if path == "/set_joint_positions":
                q = body.get("q") or []
                return self._json(200, bridge.act_set_joint_positions(list(q)))
            if path == "/set_tcp_target":
                xyz = body.get("xyz") or []
                if len(xyz) != 3:
                    return self._json(400, {"error": "xyz must have 3 entries"})
                return self._json(200, bridge.act_set_tcp_target(list(xyz)))
            if path == "/open_gripper":
                return self._json(200, bridge.act_open_gripper())
            if path == "/close_gripper":
                return self._json(200, bridge.act_close_gripper())
            if path == "/set_gripper_width":
                w = body.get("width")
                if w is None:
                    return self._json(400, {"error": "width (metres) required"})
                return self._json(200, bridge.act_set_gripper_width(float(w)))
            if path == "/grasp":
                return self._json(200, bridge.act_grasp(
                    force=body.get("force"), width=body.get("width")))
            if path == "/release":
                return self._json(200, bridge.act_release())
            if path == "/prompt":
                text = (body.get("text") or "").strip()
                if not text:
                    return self._json(400, {"error": "text required"})
                return self._json(200, bridge.act_prompt(text))
            if path == "/tool":
                # Same /tool schema the OmniSim bridges expose. Maps
                # the platform's tool-call shape onto our act_* methods.
                tool_name = (body.pop("tool", None) or "").strip()
                if not tool_name:
                    return self._json(400, {"error": "tool name required"})
                try:
                    result = _dispatch_tool(bridge, tool_name, body)
                    return self._json(200, {"status": "ok", "tool": tool_name, "result": result})
                except Exception as e:
                    return self._json(500, {"status": "err", "tool": tool_name, "error": repr(e)})
            return self._json(404, {"error": "not_found", "path": path})

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
    return method(**args)


def serve_http(bridge: BridgeBase, port: int = 8765) -> ThreadingHTTPServer:
    """Spin up the bridge HTTP server in a background thread. Returns
    the server instance so the caller can `server.shutdown()` cleanly."""
    server = ThreadingHTTPServer(("127.0.0.1", port), _make_handler(bridge))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[bridge_base] HTTP listening on http://127.0.0.1:{port}")
    return server
