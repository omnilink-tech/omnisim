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

"""Chat-router for the unified Mavic bridge.

Adds a natural-language verb surface on top of mavic_omnilink_bridge's
BridgeState so the right-click *Show Robot Window* chat panel works in
chat/omnilink_mavic.wbt — operator types "takeoff" / "forward 1 m" /
"land" and the IntentRouter mutates the same BridgeState fields the
agent-facing /action POST handler mutates. Same state, same flight loop,
two entry points.

This module is intentionally split out so the 1k-line bridge file stays
focused on the agent contract (HTTP /action + survey perception) and
this file handles the chat surface alone.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
from typing import Any, Dict, List, Optional


DEFAULT_TAKEOFF_ALTITUDE_M = 12.0  # matches mavic_omnilink_bridge.DEFAULT_TAKEOFF_ALTITUDE
DEFAULT_MOVE_DISTANCE_M = 1.0
DEFAULT_TURN_RAD = math.pi / 2


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


# ── Action helpers: mutate BridgeState the same way /action POST does ──

def _act_takeoff(state, altitude: float) -> None:
    with state.lock:
        state.target_altitude = max(0.5, altitude)
        state.target_x = state.x
        state.target_y = state.y
        state.mode = "takeoff"
        state.fault = None


def _act_land(state) -> None:
    with state.lock:
        state.target_x = state.x
        state.target_y = state.y
        state.target_altitude = 0.0
        state.mode = "land"
        state.fault = None


def _act_hover(state) -> None:
    with state.lock:
        state.target_x = state.x
        state.target_y = state.y
        if state.target_altitude < 0.5:
            state.target_altitude = max(state.z, DEFAULT_TAKEOFF_ALTITUDE_M)
        state.mode = "hover"
        state.fault = None


def _act_stop(state) -> None:
    with state.lock:
        state.target_x = None
        state.target_y = None
        state.target_yaw = None
        state.target_altitude = 0.0
        state.mode = "idle"
        state.fault = None


def _act_reset(state) -> None:
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


def _act_goto(state, x: float, y: float, altitude: Optional[float] = None) -> None:
    with state.lock:
        alt = altitude if altitude is not None else (
            state.target_altitude if state.target_altitude > 0.5
            else DEFAULT_TAKEOFF_ALTITUDE_M
        )
        state.target_x = float(x)
        state.target_y = float(y)
        state.target_altitude = max(0.5, alt)
        state.mode = "goto"
        state.fault = None


def _act_move_body(state, forward: float = 0.0, lateral: float = 0.0,
                   vertical: float = 0.0) -> None:
    """Body-frame offset: +forward = nose, +lateral = left, +vertical = up."""
    with state.lock:
        yaw = state.yaw
        x = state.x
        y = state.y
        cur_alt = state.target_altitude if state.target_altitude > 0.5 else state.z
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    world_dx = forward * cos_y - lateral * sin_y
    world_dy = forward * sin_y + lateral * cos_y
    with state.lock:
        state.target_x = x + world_dx
        state.target_y = y + world_dy
        state.target_altitude = max(0.5, cur_alt + vertical)
        state.mode = "goto"
        state.fault = None


def _act_turn(state, angle_rad: float) -> None:
    with state.lock:
        base = state.target_yaw if state.target_yaw is not None else state.yaw
        state.target_yaw = wrap_pi(base + angle_rad)


# ── IntentRouter ──────────────────────────────────────────────────────

class IntentRouter:
    NUMBER = r"(-?\d+\.?\d*)"

    def __init__(self, state) -> None:
        self.state = state

    def _distance(self, val: float, unit: Optional[str]) -> float:
        u = (unit or "m").lower()
        if u.startswith("cm") or u.startswith("centi"):
            return val / 100.0
        return val

    def dispatch(self, text: str) -> Dict[str, Any]:
        s = (text or "").strip().lower()
        if not s:
            return {"agent": "(empty prompt)", "tools": []}

        m = re.search(r"\b(takeoff|take[- ]?off|launch|lift[- ]?off)\b(?:.*?\bto\b)?[^\d-]*" + self.NUMBER + r"?\s*(m|meter|metre|meters|metres|cm)?", s)
        if m and re.search(r"\b(takeoff|take[- ]?off|launch|lift[- ]?off)\b", s):
            raw = m.group(2)
            altitude = self._distance(float(raw), m.group(3)) if raw else DEFAULT_TAKEOFF_ALTITUDE_M
            _act_takeoff(self.state, altitude)
            return {"agent": f"Taking off to {altitude:.1f} m.",
                    "tools": [("takeoff", "ok", f"alt={altitude:.1f} m")]}

        if re.search(r"\b(land|touch\s*down|descend\s+now)\b", s):
            _act_land(self.state)
            return {"agent": "Landing.", "tools": [("land", "ok", "descent")]}

        if re.search(r"\b(hover|hold(?:\s+position)?|station[- ]?keep)\b", s):
            _act_hover(self.state)
            return {"agent": "Hovering in place.", "tools": [("hover", "ok", "hold")]}

        if re.search(r"\b(stop|halt|freeze|brake)\b", s):
            _act_stop(self.state)
            return {"agent": "Stopping (drone idles). Use 'takeoff' to resume.",
                    "tools": [("stop", "ok", "idle")]}

        if re.search(r"\b(reset|home|teleport.*start|return.*spawn)\b", s):
            _act_reset(self.state)
            return {"agent": "Teleporting back to spawn.",
                    "tools": [("reset", "ok", "spawn")]}

        if re.search(r"\b(status|state|where|pose|telemetry|altitude)\b", s):
            with self.state.lock:
                x, y, z, yaw, mode = (self.state.x, self.state.y, self.state.z,
                                       self.state.yaw, self.state.mode)
            return {"agent": (f"x={x:+.2f}, y={y:+.2f}, z={z:.2f} m, "
                              f"yaw={math.degrees(yaw):+.0f}°, mode={mode}."),
                    "tools": [("get_state", "ok", mode)]}

        if re.search(r"\b(spin|rotate)\b", s) and not re.search(r"left|right|degree|deg|rad", s):
            _act_turn(self.state, 2.0 * math.pi)
            return {"agent": "Spinning 360°.", "tools": [("turn", "ok", "360°")]}

        m = re.search(r"\b(up|climb|ascend|down|descend|drop|lower)\b[^-\d]*" + self.NUMBER + r"\s*(m|meter|metre|meters|metres|cm)?", s)
        if m:
            verb = m.group(1)
            d = self._distance(float(m.group(2)), m.group(3))
            if verb in ("down", "descend", "drop", "lower"):
                d = -d
            _act_move_body(self.state, vertical=d)
            return {"agent": f"Moving {'up' if d>0 else 'down'} {abs(d):.2f} m.",
                    "tools": [("move", "ok", f"vertical={d:+.2f} m")]}
        if re.search(r"\b(up|climb|ascend)\b", s):
            _act_move_body(self.state, vertical=DEFAULT_MOVE_DISTANCE_M)
            return {"agent": f"Climbing {DEFAULT_MOVE_DISTANCE_M:.1f} m.",
                    "tools": [("move", "ok", f"vertical=+{DEFAULT_MOVE_DISTANCE_M:.1f} m")]}
        if re.search(r"\b(down|descend|drop)\b", s) and not re.search(r"\bland\b", s):
            _act_move_body(self.state, vertical=-DEFAULT_MOVE_DISTANCE_M)
            return {"agent": f"Descending {DEFAULT_MOVE_DISTANCE_M:.1f} m.",
                    "tools": [("move", "ok", f"vertical=-{DEFAULT_MOVE_DISTANCE_M:.1f} m")]}

        m = re.search(r"\bturn(?:\s+(left|right))?\s+(?:by\s+)?" + self.NUMBER + r"\s*(deg|degree|degrees|rad|radian|radians)?", s)
        if m:
            direction = m.group(1)
            val = float(m.group(2))
            unit = (m.group(3) or "deg").lower()
            angle = val if unit.startswith("rad") else math.radians(val)
            if direction == "right":
                angle = -abs(angle)
            elif direction == "left":
                angle = abs(angle)
            _act_turn(self.state, angle)
            return {"agent": f"Turning {math.degrees(angle):+.0f}°.",
                    "tools": [("turn", "ok", f"angle={math.degrees(angle):+.0f}°")]}
        m2 = re.search(r"\bturn (left|right)\b", s)
        if m2:
            direction = m2.group(1)
            sign = 1 if direction == "left" else -1
            _act_turn(self.state, sign * DEFAULT_TURN_RAD)
            return {"agent": f"Turning {direction} 90°.",
                    "tools": [("turn", "ok", f"angle={sign*90}°")]}
        if re.search(r"\b(turn around|u[- ]?turn|about face)\b", s):
            _act_turn(self.state, math.pi)
            return {"agent": "Turning around (180°).",
                    "tools": [("turn", "ok", "angle=180°")]}

        m = re.search(r"\bgo\s*to\s+" + self.NUMBER + r"[,\s]+" + self.NUMBER +
                       r"(?:\s+(?:at|alt|altitude)\s+" + self.NUMBER + r"\s*(m|meters)?)?", s)
        if m:
            tx = float(m.group(1)); ty = float(m.group(2))
            alt_str = m.group(3)
            alt = float(alt_str) if alt_str else None
            _act_goto(self.state, tx, ty, alt)
            return {"agent": f"Flying to ({tx:+.2f}, {ty:+.2f})" + (f" at {alt:.1f} m." if alt else "."),
                    "tools": [("goto", "ok", f"({tx:+.2f}, {ty:+.2f})")]}

        m = re.search(r"\b(?:strafe\s+)?(left|right)\b[^-\d]*" + self.NUMBER + r"\s*(m|meter|metre|meters|metres|cm)?", s)
        if m and not re.search(r"\bturn\b", s):
            direction = m.group(1)
            d = self._distance(float(m.group(2)), m.group(3))
            lat = d if direction == "left" else -d
            _act_move_body(self.state, lateral=lat)
            return {"agent": f"Strafing {direction} {abs(lat):.2f} m.",
                    "tools": [("move", "ok", f"lateral={lat:+.2f} m")]}

        m = re.search(r"\b(forward|forwards|ahead|fly forward|back|backward|backwards|reverse|fly back)\b[^-\d]*" + self.NUMBER + r"\s*(m|meter|metre|meters|metres|cm)?", s)
        if m:
            direction = m.group(1).replace("fly ", "")
            d = self._distance(float(m.group(2)), m.group(3))
            if direction.startswith("back") or direction == "reverse":
                d = -d
            _act_move_body(self.state, forward=d)
            return {"agent": f"Flying {direction} {abs(d):.2f} m.",
                    "tools": [("move", "ok", f"forward={d:+.2f} m")]}
        if re.search(r"\b(forward|forwards|ahead)\b", s):
            _act_move_body(self.state, forward=DEFAULT_MOVE_DISTANCE_M)
            return {"agent": f"Flying forward {DEFAULT_MOVE_DISTANCE_M:.1f} m.",
                    "tools": [("move", "ok", f"forward=+{DEFAULT_MOVE_DISTANCE_M:.1f} m")]}
        if re.search(r"\b(back|reverse)\b", s):
            _act_move_body(self.state, forward=-DEFAULT_MOVE_DISTANCE_M)
            return {"agent": f"Flying back {DEFAULT_MOVE_DISTANCE_M:.1f} m.",
                    "tools": [("move", "ok", f"forward=-{DEFAULT_MOVE_DISTANCE_M:.1f} m")]}

        return {
            "agent": ("I don't recognise that. Try: \"takeoff\", \"forward 1 m\", "
                      "\"up 2 m\", \"left 1 m\", \"turn right 90 degrees\", "
                      "\"goto 2 3\", \"hover\", \"land\", \"stop\", \"reset\"."),
            "tools": [],
        }


# ── wwi plumbing ──────────────────────────────────────────────────────

def queue_window(state, line: str) -> None:
    with state.lock:
        state.window_outbox.append(line)


def push_configure(state) -> None:
    cfg = {
        "robot": "DJI Mavic 2 Pro",
        "robot_class": "aerial drone",
        "agent": "local intent (regex)",
        "suggestions": [
            "takeoff",
            "forward 1 m",
            "up 1 m",
            "turn left 90 degrees",
            "hover",
            "land",
        ],
    }
    queue_window(state, "configure:" + json.dumps(cfg))
    queue_window(state, "status:connected")
    with state.lock:
        state.window_configured = True


def handle_wwi_message(state, router: IntentRouter, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(state)
        return
    if msg.startswith("stop"):
        _act_stop(state)
        queue_window(state, "agent:Stop received — motors idled.")
        queue_window(state, "tool:stop:ok:idle")
        queue_window(state, "status:idle")
        return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        queue_window(state, "status:thinking")
        result = router.dispatch(text)
        for (tool, status, summary) in result["tools"]:
            queue_window(state, f"tool:{tool}:{status}:{summary}")
        queue_window(state, "agent:" + result["agent"])
        queue_window(state, "status:idle")
        return
    queue_window(state, "system:Unknown window message: " + msg[:200])
