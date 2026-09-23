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
chat/omnilink_mavic.omniworld — operator types "takeoff" / "forward 1 m" /
"land" and the shared deterministic parser (via `_StateBridge` below)
mutates the same BridgeState fields the agent-facing /action POST handler
mutates. Same state, same flight loop, two entry points.

This module is intentionally split out so the 1k-line bridge file stays
focused on the agent contract (HTTP /action + survey perception) and
this file handles the chat surface alone.
"""

from __future__ import annotations

import json
import math


DEFAULT_TAKEOFF_ALTITUDE_M = 12.0  # matches mavic_omnilink_bridge.DEFAULT_TAKEOFF_ALTITUDE


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
        # The pose the WORLD authored, exactly as `POST /action reset` uses
        # (public issue #14). Until 2026-09-23 this path still hard-coded the
        # omnilink_mavic spawn (0, -12, yaw 90 deg): on the arena draft the
        # model's reset_to_home put the aircraft 12 m off the floor plan.
        home = dict(getattr(state, "authored_pose", None)
                    or {"x": 0.0, "y": -12.0, "z": 0.1, "yaw": math.pi / 2})
        state.reset_request = home
        state.reset_anchor = dict(home)
        state.xy_i_fwd = 0.0
        state.xy_i_right = 0.0


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


# ── The chat surface ──────────────────────────────────────────────────

class _StateBridge:
    """Adapts this module's _act_* functions to the act_* attribute API the
    rest of the stack expects -- `omnisim_bridges.route` and the drone's
    `/tool` registry both drive the aircraft through this one object, so the
    drone shares one vocabulary with the tug, the arm and the quadruped
    instead of keeping its own keyword ladder.

    The ladder it replaced had the same class of defect as the others, and
    one that is worse on a drone: `land` is an ordinary English word, so
    "where did the package land?" used to LAND THE AIRCRAFT. The parser
    settles question form before any rule is tried.
    """

    def __init__(self, state) -> None:
        self.state = state

    # ── Telemetry forwarded from the state object (plan D1 / D4) ──────
    #
    # `route.execute` and the event dispatcher read `events`, `sim_time`,
    # `sim_step`, `robot_id` and `fault` off whatever bridge object they are
    # handed, with `getattr(..., default)`. This adapter IS that object on
    # the drone, so without these five properties a gate refusal on the
    # parser path would be filed against a ring that does not exist and a
    # `sim_time` of 0.0 -- an event stamped at the beginning of time, which
    # is worse than no event because it looks like a real one.

    @property
    def events(self):
        return getattr(self.state, "events", None)

    @property
    def sim_time(self) -> float:
        return float(getattr(self.state, "sim_time", 0.0) or 0.0)

    @property
    def sim_step(self) -> int:
        return int(getattr(self.state, "sim_step", 0) or 0)

    @property
    def robot_id(self) -> str:
        return str(getattr(self.state, "robot_id", "mavic2pro") or "mavic2pro")

    @property
    def fault(self):
        return getattr(self.state, "fault", None)

    @property
    def surface(self) -> str:
        return "drone"

    def act_takeoff(self, altitude=None):
        _act_takeoff(self.state, DEFAULT_TAKEOFF_ALTITUDE_M
                     if altitude is None else float(altitude))
        return {"accepted": True}

    def act_land(self):
        _act_land(self.state)
        return {"accepted": True}

    def act_hover(self):
        _act_hover(self.state)
        return {"accepted": True}

    def act_stop(self, source="external"):
        _act_stop(self.state)
        return {"accepted": True}

    def act_reset_to_home(self):
        _act_reset(self.state)
        return {"accepted": True}

    def act_turn(self, angle_rad):
        _act_turn(self.state, float(angle_rad))
        return {"accepted": True}

    def act_move_body(self, forward=None, vertical=None):
        _act_move_body(self.state, forward=float(forward or 0.0),
                       vertical=float(vertical or 0.0))
        return {"accepted": True}

    def get_state_for_query(self):
        with self.state.lock:
            return {"x": self.state.x, "y": self.state.y,
                    "z": self.state.z, "mode": self.state.mode}

    def describe_state(self):
        """The drone's own sentence. The shared describer knows arms and
        tugs, and would answer "how high are you?" with "in idle mode"."""
        with self.state.lock:
            x, y, z = self.state.x, self.state.y, self.state.z
            yaw, mode = self.state.yaw, self.state.mode
        return (f"x={x:+.2f}, y={y:+.2f}, altitude={z:.2f} m, "
                f"yaw={math.degrees(yaw):+.0f} deg, mode={mode}.")


# ── wwi plumbing ──────────────────────────────────────────────────────

def queue_window(state, line: str) -> None:
    with state.lock:
        state.window_outbox.append(line)


def relay_event_to_window(state, kind: str, payload: dict) -> None:
    """Render one relay turn event as robot-window protocol lines.

    Module-level, because plan D4's WAKE uses the SAME sink an operator's
    typed turn uses: a wake IS an ordinary turn -- same cascade, same gate,
    same memory write -- and the only thing that differs is who wrote the
    sentence. A second, wake-only renderer would be a second place for the
    chat panel's protocol to drift.

    Safe from any thread: `queue_window` appends under the state's own lock
    and the SIM THREAD drains the outbox. It touches no Robot API.
    """
    if kind == "status":
        queue_window(state, "status:" + str(payload.get("state", "idle")))
    elif kind in ("agent", "error"):
        queue_window(state, kind + ":" + str(payload.get("text", "")))
    elif kind == "tool":
        queue_window(state,
                     f"tool:{payload.get('name', '?')}:"
                     f"{payload.get('status', 'ok')}:"
                     f"{payload.get('summary', '')}")


def push_configure(state) -> None:
    from omnisim_bridges.access import chat_config
    relay = getattr(state, "omnilink_relay", None)
    cfg = {
        "robot": "DJI Mavic 2 Pro",
        "robot_class": "aerial drone",
        "agent": "OmniLink" if relay is not None else "OmniLink connection required",
        **chat_config(relay),
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


def _parser_first_window(state, text: str, to_model, spawn=None) -> bool:
    """Parser-first on the robot-window path. True = the turn is taken.

    ⚠️ handle_wwi_message is drained on the SIM THREAD, so the shared
    helper decides here (pure regex) and executes on a worker.

    ⚠️ The caller checks `relay is None` FIRST and refuses. This is never
    reached without an OmniKey, and must never be made reachable without
    one -- see the 2026-09-22 access policy.
    """
    if getattr(state, "omnilink_relay", None) is None:
        return False                       # belt and braces; see above
    try:
        from omnisim_bridges.route import parser_first_window
    except ImportError:                                # pragma: no cover
        return False
    try:
        return bool(parser_first_window(
            _StateBridge(state), text, "drone",
            lambda line: queue_window(state, line), to_model, spawn=spawn))
    except Exception as exc:               # never take the demo down
        print(f"[mavic_chat_router] parser-first skipped: {exc!r}", flush=True)
        return False


def handle_wwi_message(state, msg: str) -> None:
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
        from omnisim_bridges.access import connection_error
        relay = getattr(state, "omnilink_relay", None)
        # ⚠️ ACCESS CHECK FIRST, AND NOTHING IS INTERPRETED BEFORE IT.
        # The parser below needs no key and no network, which is exactly
        # why it must sit BEHIND this: a keyless /prompt is refused, full
        # stop (2026-09-22 access policy).
        if relay is None:
            queue_window(state, "error:" + connection_error()["response"])
            queue_window(state, "status:error")
            return
        text = msg[len("prompt:"):]

        def on_event(kind, payload):
            relay_event_to_window(state, kind, payload)

        def _to_model():
            relay.dispatch_async(text, on_event)

        # PARSER FIRST. The window path shares the aircraft's one
        # vocabulary with /prompt, so "climb 2 m" typed into the panel is
        # judged on the same "drone" rail (gate.MAX_ALTITUDE_M) the HTTP
        # route uses -- and answered without paying for a model.
        if _parser_first_window(state, text, _to_model):
            return
        _to_model()
        return
    queue_window(state, "system:Unknown window message: " + msg[:200])
