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

"""omnilink_quadruped_bridge — Spot-specific bridge.

This bridge exposes the same soft-drop / crouch / stand sequence as
`spot_simple_pose` plus an intent surface:

    stand       -- hold the standing stance with a tiny sway
    sit         -- crouch low (knees bent further)
    wave        -- exaggerate the sway for ~6 s
    walk        -- wave-gait leg cycle + supervisor-driven forward
                   body translation (the only way to keep Spot
                   upright while it locomotes -- pure-physics walking
                   on this URDF reliably topples after 5-10 s; see
                   `spot_simple_pose.py`)
    stop        -- freeze the legs in current position
    home        -- alias for stand

The HTTP surface is intentionally minimal: list_robots, get_robot_state,
stop_robot, reset_to_home, prompt. set_joint_positions is also exposed
for advanced users who want to send raw joint vectors.

Robot window: same omnilink_chat plugin.
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
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

try:
    from _omnilink_relay import OmniLinkRelay, Tool, is_enabled as omnilink_enabled, get_omni_key
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


LEGS = ("front_left", "front_right", "rear_left", "rear_right")

# Stance constants -- copied from spot_simple_pose.
HIP_X_LEFT = 0.30
HIP_X_RIGHT = -0.30
HIP_Y_STAND = 0.30
KNEE_STAND = -0.60
HIP_Y_SIT = 0.55
KNEE_SIT = -1.20
EXTEND_HIP_Y = 0.15
EXTEND_KNEE = -0.30

# Walk parameters (wave gait + supervisor-driven body translation; see
# spot_simple_pose for the longer rationale and physics caveats).
WALK_PERIOD_S = 4.0
SWING_FRACTION = 0.20
SWING_KNEE_DELTA = -0.30
HIP_SWEEP_AMP = 0.18
WALK_VELOCITY_MS = 0.30        # forward translation speed during walk
WALK_VELOCITY_RAMP_S = 2.0     # accelerate from 0 to WALK_VELOCITY_MS
COM_SHIFT_AMP_Y = 0.08

LEG_PHASES = {
    "front_right": 0.00,
    "rear_left":   0.25,
    "front_left":  0.50,
    "rear_right":  0.75,
}


def hip_x_for(leg: str) -> float:
    return HIP_X_LEFT if "left" in leg else HIP_X_RIGHT


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="spot")
    p.add_argument("--port", type=int, default=8765)
    args, _ = p.parse_known_args()
    return args


class QuadrupedBridge:
    def __init__(self, robot: Supervisor, robot_id: str = "spot") -> None:
        self.robot = robot
        self.robot_id = robot_id
        self.timestep = int(robot.getBasicTimeStep())

        # 12 motors: <leg>_<hip_x|hip_y|knee>_motor
        self.motors = {}
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                name = f"{leg}_{joint}_motor"
                m = robot.getDevice(name)
                if m is None:
                    print(f"[omnilink_quadruped_bridge] WARNING: missing {name}")
                self.motors[(leg, joint)] = m

        # Motion state.
        self.lock = threading.RLock()
        # ("settle"|"crouch"|"stand"|"sit"|"wave"|"walk"|"stop", params)
        self.motion = ("settle", {"t0": robot.getTime()})
        self.last_tick_at = time.time()

        # Supervisor handles for walk-phase body translation + orientation lock.
        self.self_node = None
        self.translation_field = None
        self.rotation_field = None
        try:
            self.self_node = robot.getSelf()
        except Exception:
            self.self_node = None
        if self.self_node is not None:
            try:
                self.translation_field = self.self_node.getField("translation")
            except Exception:
                self.translation_field = None
            try:
                self.rotation_field = self.self_node.getField("rotation")
            except Exception:
                self.rotation_field = None

        # Persistent floating-base anchor (x, y, z), captured lazily on the
        # first body-lock. The body is supervisor-pinned to this every tick so
        # the stiff-legged stance holds upright under any physics backend
        # (without it the unbalanced stance topples under Newton/XPBD; ODE's
        # softer solver merely tolerated it).
        self.body_anchor: Optional[Tuple[float, float, float]] = None

        # wwi outbox.
        self.window_outbox: List[str] = []
        self.window_configured = False

        self.capabilities = {
            "legs": list(LEGS),
            "joints_per_leg": ["hip_x", "hip_y", "knee"],
            "stand_pose": {"hip_y": HIP_Y_STAND, "knee": KNEE_STAND},
            "sit_pose": {"hip_y": HIP_Y_SIT, "knee": KNEE_SIT},
            "walk_velocity_ms": WALK_VELOCITY_MS,
        }

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    def _set_all(self, hip_y: float, knee: float, hip_x_extra: float = 0.0) -> None:
        for leg in LEGS:
            base = hip_x_for(leg)
            mx = self.motors[(leg, "hip_x")]
            my = self.motors[(leg, "hip_y")]
            mk = self.motors[(leg, "knee")]
            if mx is not None:
                mx.setPosition(base + hip_x_extra)
            if my is not None:
                my.setPosition(hip_y)
            if mk is not None:
                mk.setPosition(knee)

    def _ensure_anchor(self) -> None:
        """Capture the floating-base anchor once, from the live body pose."""
        if self.body_anchor is not None:
            return
        pos0 = None
        if self.self_node is not None:
            try:
                pos0 = self.self_node.getPosition()
            except Exception:
                pos0 = None
        if pos0 is not None and len(pos0) >= 3:
            self.body_anchor = (float(pos0[0]), float(pos0[1]), float(pos0[2]))
        else:
            self.body_anchor = (0.0, 0.0, 0.85)

    def _write_body_pose(self, x: float, y: float, z: float) -> None:
        """Rewrite the floating-base translation + level orientation.

        Only the body link's pose is rewritten; the leg joints are NOT
        zeroed (that would need Node.resetPhysics), so position-controlled
        gaits/poses keep animating underneath.
        """
        if self.translation_field is None:
            return
        try:
            self.translation_field.setSFVec3f([x, y, z])
            if self.rotation_field is not None:
                self.rotation_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
        except Exception:
            pass

    def _lock_body_upright(self, z_extra: float = 0.0) -> None:
        """Hold the body at its anchor, level — the same supervisor pose-lock
        _tick_walk uses, applied to the standing/idle poses too. Without it the
        stiff-legged stance has no balance feedback and topples under
        Newton/XPBD (it stood only because ODE's softer solver tolerated it)."""
        if self.self_node is None or self.translation_field is None:
            return
        self._ensure_anchor()
        ax, ay, az = self.body_anchor
        self._write_body_pose(ax, ay, az + z_extra)

    def tick(self, sim_t: float) -> None:
        with self.lock:
            kind, p = self.motion
            self.last_tick_at = time.time()

        if kind == "settle":
            # Hold extended for 1.5 s for soft drop.
            self._set_all(EXTEND_HIP_Y, EXTEND_KNEE)
            if sim_t - p["t0"] > 1.5:
                with self.lock:
                    self.motion = ("crouch", {"t0": sim_t})
        elif kind == "crouch":
            # Ramp into standing stance over 3 s.
            phase = min(1.0, (sim_t - p["t0"]) / 3.0)
            hy = EXTEND_HIP_Y + (HIP_Y_STAND - EXTEND_HIP_Y) * phase
            kn = EXTEND_KNEE + (KNEE_STAND - EXTEND_KNEE) * phase
            self._set_all(hy, kn)
            if phase >= 1.0:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "stand":
            sway = 0.04 * math.sin(2 * math.pi * (sim_t - p["t0"]) / 3.0)
            self._set_all(HIP_Y_STAND + sway, KNEE_STAND)
        elif kind == "sit":
            self._set_all(HIP_Y_SIT, KNEE_SIT)
        elif kind == "wave":
            t = sim_t - p["t0"]
            sway = 0.10 * math.sin(2 * math.pi * 0.8 * t)
            self._set_all(HIP_Y_STAND + sway, KNEE_STAND, hip_x_extra=sway * 0.5)
            if t > p["duration_s"]:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "walk":
            self._tick_walk(sim_t, p)
        elif kind == "stop":
            # Do nothing -- last setpoints stay applied.
            pass

        # Pin the floating base upright for every non-walk pose (walk runs its
        # own translation+rotation lock in _tick_walk). This is what keeps Spot
        # standing under Newton/XPBD; see _lock_body_upright.
        if kind != "walk":
            self._lock_body_upright()

    def _tick_walk(self, sim_t: float, p: Dict[str, Any]) -> None:
        """Joint-space wave gait + supervisor-driven forward translation.

        Each leg cycles realistically through stance/swing (animated by
        position-controlled motors). The body's X position is locked to
        a straight forward line from the walk-start anchor, advancing at
        WALK_VELOCITY_MS m/s (ramped from 0). Pure physics walking is
        not stable on this URDF; supervisor translation lets Spot
        visibly walk while keeping the gait visually realistic.
        """
        walk_t = sim_t - p["t0"]
        cycle_phase = (walk_t / WALK_PERIOD_S) % 1.0
        amp_scale = max(0.0, min(1.0, walk_t / (2.0 * WALK_PERIOD_S)))

        # Lateral CoM shift via hip_x delta during single-leg swing.
        com_shift_y = -COM_SHIFT_AMP_Y * math.cos(2.0 * math.pi * cycle_phase) * amp_scale

        for leg in LEGS:
            offset = (cycle_phase - LEG_PHASES[leg]) % 1.0
            if offset < SWING_FRACTION:
                # Swing: hip_y descends from STANCE+A to STANCE-A; knee flexes.
                pp = offset / SWING_FRACTION
                smooth = pp * pp * (3.0 - 2.0 * pp)
                hip_y = HIP_Y_STAND + HIP_SWEEP_AMP * (1.0 - 2.0 * smooth) * amp_scale
                knee = KNEE_STAND + SWING_KNEE_DELTA * math.sin(math.pi * pp) * amp_scale
            else:
                # Stance: hip_y rises from STANCE-A to STANCE+A; foot stays
                # planted -> body moves forward in world.
                pp = (offset - SWING_FRACTION) / (1.0 - SWING_FRACTION)
                hip_y = HIP_Y_STAND + HIP_SWEEP_AMP * (-1.0 + 2.0 * pp) * amp_scale
                knee = KNEE_STAND

            base = hip_x_for(leg)
            mx = self.motors[(leg, "hip_x")]
            my = self.motors[(leg, "hip_y")]
            mk = self.motors[(leg, "knee")]
            if mx is not None:
                mx.setPosition(base + com_shift_y)
            if my is not None:
                my.setPosition(hip_y)
            if mk is not None:
                mk.setPosition(knee)

        # Advance the shared body anchor along +x by this tick's ramped
        # distance, then write it (level). Using the persistent anchor (rather
        # than a per-walk start offset) means a later 'stand' holds the
        # advanced position instead of snapping back to the spawn point.
        if self.self_node is not None and self.translation_field is not None:
            self._ensure_anchor()
            v_cmd = WALK_VELOCITY_MS * max(0.0, min(1.0, walk_t / WALK_VELOCITY_RAMP_S))
            dt = max(0.0, self.timestep / 1000.0)
            ax, ay, az = self.body_anchor
            ax += v_cmd * dt
            self.body_anchor = (ax, ay, az)
            body_bob = 0.015 * math.cos(4.0 * math.pi * cycle_phase) * amp_scale
            self._write_body_pose(ax, ay, az + body_bob)

    # ── Actions ──────────────────────────────────────────────────

    def act_stop(self) -> dict:
        with self.lock:
            self.motion = ("stop", {})
        return {"halted_at": time.time()}

    def act_stand(self) -> dict:
        with self.lock:
            self.motion = ("stand", {"t0": self.robot.getTime()})
        return {"accepted": True, "pose": "stand"}

    def act_sit(self) -> dict:
        with self.lock:
            self.motion = ("sit", {})
        return {"accepted": True, "pose": "sit"}

    def act_wave(self, duration_s: float = 6.0) -> dict:
        with self.lock:
            self.motion = ("wave", {"t0": self.robot.getTime(), "duration_s": duration_s})
        return {"accepted": True, "duration_s": duration_s}

    def act_walk(self) -> dict:
        with self.lock:
            self.motion = ("walk", {"t0": self.robot.getTime()})
        return {"accepted": True, "pose": "walk", "velocity_ms": WALK_VELOCITY_MS}

    def act_reset_to_home(self) -> dict:
        return self.act_stand()

    def get_state(self) -> dict:
        with self.lock:
            kind = self.motion[0]
        return {
            "id": self.robot_id,
            "model": "Boston Dynamics Spot",
            "mode": kind,
            "last_tick_at": self.last_tick_at,
            "sim_time": self.robot.getTime(),
        }


# ── Intent ───────────────────────────────────────────────────────────

class IntentRouter:
    def __init__(self, bridge: QuadrupedBridge):
        self.bridge = bridge

    def dispatch(self, text: str) -> dict:
        s = text.strip().lower()
        if re.search(r"\b(stop|halt|freeze)\b", s):
            self.bridge.act_stop()
            return {"agent": "Holding position.", "tools": [("stop_robot", "ok", "frozen")]}
        if re.search(r"\b(sit|crouch|down|low)\b", s):
            self.bridge.act_sit()
            return {"agent": "Sitting down.", "tools": [("set_joint_positions", "ok", "sit pose")]}
        if re.search(r"\b(stand|up|home|reset)\b", s):
            self.bridge.act_stand()
            return {"agent": "Standing.", "tools": [("reset_to_home", "ok", "stand pose")]}
        if re.search(r"\b(wave|hello|dance|demo|show)\b", s):
            self.bridge.act_wave()
            return {"agent": "Waving — give me ~6 seconds.", "tools": [("wave", "ok", "0.8 Hz sway")]}
        if re.search(r"\b(walk|forward|move|go)\b", s):
            self.bridge.act_walk()
            return {
                "agent": f"Walking forward at {WALK_VELOCITY_MS:.2f} m/s.",
                "tools": [("walk", "ok", f"v={WALK_VELOCITY_MS} m/s")],
            }
        if re.search(r"\b(status|state|where|pose)\b", s):
            st = self.bridge.get_state()
            return {"agent": f"mode={st['mode']}", "tools": [("get_robot_state", "ok", st["mode"])]}
        return {
            "agent": "Try: \"stand\", \"sit\", \"wave hello\", \"stop\".",
            "tools": [],
        }


# ── HTTP ─────────────────────────────────────────────────────────────

def make_handler(bridge: QuadrupedBridge, router: IntentRouter, relay: Any = None):
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
                    "id": bridge.robot_id, "model": "Boston Dynamics Spot",
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
                    "id": bridge.robot_id, "model": "Boston Dynamics Spot",
                    "capabilities": bridge.capabilities,
                }])
            if p == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if p == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if p == "/prompt":
                text = (body.get("text") or "").strip()
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
                # calls on its side; dispatch via the relay-registered Tool.
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


def start_http(bridge: QuadrupedBridge, router: IntentRouter, port: int, relay: Any = None):
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, router, relay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[omnilink_quadruped_bridge] HTTP on http://127.0.0.1:{port}")


def build_quadruped_tools(bridge: QuadrupedBridge) -> List[Any]:
    if Tool is None:
        return []
    return [
        Tool(name="stand", description="Hold a standing stance with gentle sway.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stand()),
        Tool(name="sit", description="Crouch / sit pose (knees bent further).",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_sit()),
        Tool(name="wave", description="Exaggerated sway for ~6 s -- 'hello' gesture.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_wave()),
        Tool(name="walk", description=(
                "Walk forward at " + f"{WALK_VELOCITY_MS:.2f}" + " m/s with a "
                "wave-gait leg cycle. Continues until 'stand' or 'stop' is called."),
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_walk()),
        Tool(name="stop_robot", description="Freeze the legs at their current pose.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stop()),
        Tool(name="get_robot_state", description="Read current pose mode.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.get_state()),
    ]


def setup_omnilink_relay(bridge: QuadrupedBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{bridge.robot_id}"
        tools = build_quadruped_tools(bridge)
        main_task = (
            "You operate Spot in OmniSim through the OmniLink bridge. "
            "Available actions: stand, sit, wave, walk (forward, ~0.3 m/s), "
            "stop_robot. Translate operator requests into one tool call. "
            "Keep responses short."
        )
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
        )
        # Push a Spot profile so operators can chat to it from the
        # omnilink-agents.com web UI; tool calls round-trip via /tool.
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
        print(f"[omnilink_quadruped_bridge] OmniLink relay ON (agent='{agent_name}')")
        return relay
    except Exception as e:
        print(f"[omnilink_quadruped_bridge] OmniLink relay setup failed: {e}")
        return None


def push_configure(bridge: QuadrupedBridge, relay: Any) -> None:
    agent_label = (
        f"OmniLink relay ({_os.environ.get('OMNILINK_ENGINE', 'g4-engine')})"
        if relay is not None else "local intent (regex)"
    )
    cfg = {
        "robot": "Spot",
        "robot_class": "quadruped",
        "agent": agent_label,
        "suggestions": ["stand", "sit", "wave hello", "stop"],
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def _on_relay_event(bridge: QuadrupedBridge, kind: str, payload: Dict[str, Any]) -> None:
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


def handle_wwi(bridge: QuadrupedBridge, router: IntentRouter, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay); return
    if msg.startswith("stop"):
        bridge.act_stop()
        bridge.queue_window("agent:Stop received.")
        bridge.queue_window("tool:stop_robot:ok:frozen")
        bridge.queue_window("status:idle"); return
    if msg.startswith("prompt:"):
        text = msg[len("prompt:"):]
        if relay is not None:
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))
            return
        bridge.queue_window("status:thinking")
        result = router.dispatch(text)
        for (t, st, sm) in result["tools"]:
            bridge.queue_window(f"tool:{t}:{st}:{sm}")
        bridge.queue_window("agent:" + result["agent"])
        bridge.queue_window("status:idle"); return
    if msg.startswith("audio_in:"):
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)"); return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}"); return
        bridge.queue_window("status:transcribing")

        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle"); return
            bridge.queue_window("transcript:" + text)
            relay.dispatch_async(text, lambda k, p: _on_relay_event(bridge, k, p))

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return


def main() -> int:
    args = _parse_args()
    robot = Supervisor()
    bridge = QuadrupedBridge(robot, args.robot)
    router = IntentRouter(bridge)
    relay = setup_omnilink_relay(bridge, http_port=args.port)
    start_http(bridge, router, args.port, relay)
    print(f"[omnilink_quadruped_bridge] Spot ready ({'OmniLink' if relay else 'local'}).")

    timestep = bridge.timestep
    while robot.step(timestep) != -1:
        sim_t = robot.getTime()
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi(bridge, router, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception:
                pass
        bridge.tick(sim_t)
    return 0


if __name__ == "__main__":
    sys.exit(main())
