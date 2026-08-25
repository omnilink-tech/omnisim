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

# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""Husky unit agent -- a dedicated OmniLink agent for ONE Husky.

One process per robot. The swarm coordinator (swarm_agent.py, agent
"HuskySwarm") is the foreman; it delegates single-robot work to these
unit agents via the OmniLink platform's delegate_to_agent. Each unit
agent owns exactly one Husky: its husky id is baked into every tool,
so the model driving it cannot address any other robot.

Run:

    export OMNI_KEY="olink_..."
    python agents/production/husky_swarm/husky_unit_agent.py --husky ne --port 51521

Reuse policy (deliberate, see the task notes):

  - IMPORTED from swarm_agent: _parse_bridges_env (pure env parser -- the
    unit agent must resolve the SAME bridge URL for its husky as the
    coordinator does) and ensure_profile (pure create-or-update against a
    passed-in client). Importing swarm_agent is clean: its module level
    only parses env + defines functions; no server, no DB init.
  - COPIED (adapted, ~30 lines each) rather than imported: _bridge_post and
    the presence heartbeat. Both close over swarm_agent's OWN module globals
    (its _ESTOP latch, its BRIDGES map, its 40-tool count) -- importing them
    would gate this agent's motion on a *different module's* e-stop that
    nothing in this process ever sets, and advertise the wrong tool count in
    presence. The copies here are wired to THIS module's state.

Self-test (no network, no OMNI_KEY, bridge stubbed):

    python husky_unit_agent.py --husky ne --self-test
"""

from __future__ import annotations

import argparse
import collections
import http.server
import json
import math
import os
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "packages" / "omnisim-bridges" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from omnisim_bridges.http_security import (  # noqa: E402
    RequestError,
    bearer_token,
    checked_origin,
    error_envelope,
    read_json,
    require_authorized,
    trusted_origins_from_env,
    validate_bind_host,
)

# Same UTF-8 discipline as swarm_agent: honor PYTHONIOENCODING when set,
# otherwise force utf-8 with backslashreplace so Windows consoles never crash.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    except Exception:
        pass

try:
    import truststore  # type: ignore
    truststore.inject_into_ssl()
except Exception:
    pass

# swarm_agent hard-exits at import when omnilink-lib is missing, which is
# also this agent's own requirement -- so the import doubles as the check.
from swarm_agent import _parse_bridges_env, ensure_profile  # noqa: E402


# --- Configuration ---------------------------------------------------

# Match swarm_agent's platform base URL resolution (OMNILINK_BASE_URL,
# default https://www.omnilink-agents.com); OMNILINK_API_BASE is accepted
# as an alias per the unit-agent contract.
BASE_URL = (os.environ.get("OMNILINK_API_BASE", "").strip()
            or os.environ.get("OMNILINK_BASE_URL", "").strip()
            or "https://www.omnilink-agents.com")

BRIDGE_TIMEOUT_S = float(os.environ.get("HUSKY_SWARM_BRIDGE_TIMEOUT_S", "10"))
PRESENCE_INTERVAL_S = float(os.environ.get("HUSKY_UNIT_PRESENCE_INTERVAL_S", "5"))
BRIDGE_TOKEN = os.environ.get("OMNISIM_BRIDGE_TOKEN", "").strip()
UNIT_HOST = os.environ.get("HUSKY_UNIT_HOST", "127.0.0.1").strip() or "127.0.0.1"

DRIVE_CLAMP_M = 10.0
TURN_CLAMP_DEG = 180.0
DRIVE_TO_XY_MAX_M = 30.0

# Open-loop gain compensation, same measured values swarm_agent uses.
TURN_GAIN = float(os.environ.get("HUSKY_SWARM_TURN_GAIN", "0.813"))
DRIVE_GAIN = float(os.environ.get("HUSKY_SWARM_DRIVE_GAIN", "0.905"))

DEFAULT_UNIT_PORTS = {"ne": 51521, "nw": 51522, "se": 51523, "sw": 51524}

# Set by _configure() from the CLI before anything runs.
HUSKY_ID = ""            # short id, e.g. "ne"
HUSKY_KEY = ""           # bridge key, e.g. "husky_ne"
AGENT_NAME = ""          # e.g. "HuskyNE"
BRIDGE_URL = ""          # e.g. "http://127.0.0.1:8865"
LOG_PREFIX = "[husky_unit]"

_START_TS = time.time()

# Motion tool names (bridge paths are gated inside _bridge_post; tool-level
# preflight uses this set too).
MOTION_TOOLS = {"drive_forward", "turn", "drive_to_xy"}
MOTION_PATHS = {"drive_forward", "turn", "set_velocity", "reset_to_home"}


def _log(msg: str) -> None:
    print(f"{LOG_PREFIX} {msg}", flush=True)


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _configure(husky: str, port: Optional[int]) -> int:
    """Resolve identity + bridge URL from the CLI husky id. Returns the port."""
    global HUSKY_ID, HUSKY_KEY, AGENT_NAME, BRIDGE_URL, LOG_PREFIX
    hid = husky.strip().lower()
    if hid.startswith("husky_"):
        hid = hid[len("husky_"):]
    HUSKY_ID = hid
    HUSKY_KEY = f"husky_{hid}"
    AGENT_NAME = "Husky" + hid.upper()
    LOG_PREFIX = f"[{AGENT_NAME}]"
    bridges = _parse_bridges_env()
    BRIDGE_URL = bridges.get(HUSKY_KEY, "") or bridges.get(hid, "")
    if not BRIDGE_URL:
        raise SystemExit(
            f"{LOG_PREFIX} no bridge URL for {HUSKY_KEY!r}. Known: {sorted(bridges)}. "
            f"Set HUSKY_SWARM_BRIDGES or pass a known husky id.")
    if port is not None:
        return port
    return DEFAULT_UNIT_PORTS.get(hid, 51521)


# --- E-stop (per-unit latch; operator surface, never a model tool) ----
#
# The swarm coordinator's e-stop lives in ITS process; there is no shared
# store between the two, so this agent keeps its OWN latch, exposed at
# POST /estop with the same wire shape as the coordinator's. The runner /
# operator UI that engages the swarm e-stop is expected to fan the same
# POST out to the unit agents. Additionally, every motion tool preflights
# the bridge's live fault field, so a robot the bridge has faulted is
# refused regardless of any latch.

_ESTOP = threading.Event()
_ESTOP_REASON = ""


def estop_engage(reason: str = "operator") -> Dict[str, Any]:
    global _ESTOP_REASON
    _ESTOP_REASON = reason
    _ESTOP.set()
    try:
        _bridge_post("stop_robot", {}, timeout=4, _bypass_estop=True)
    except Exception:
        pass
    return {"estop": "engaged", "reason": reason, "husky": HUSKY_KEY}


def estop_clear() -> Dict[str, Any]:
    global _ESTOP_REASON
    _ESTOP.clear()
    _ESTOP_REASON = ""
    return {"estop": "cleared", "husky": HUSKY_KEY}


def estop_state() -> Dict[str, Any]:
    return {"engaged": _ESTOP.is_set(), "reason": _ESTOP_REASON}


# --- Bridge plumbing (adapted copy of swarm_agent._bridge_post) -------

def _bridge_post(path: str, payload: Optional[Dict[str, Any]] = None,
                 timeout: Optional[float] = None,
                 _bypass_estop: bool = False) -> Dict[str, Any]:
    """POST to THIS husky's bridge. E-stop gates the motion paths."""
    if not BRIDGE_URL:
        return {"error": "bridge URL not configured", "husky": HUSKY_KEY}
    if (not _bypass_estop) and _ESTOP.is_set() and path.lstrip("/") in MOTION_PATHS:
        return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": HUSKY_KEY,
                "hint": "Motion is disabled by the operator e-stop. Report this "
                        "and stop trying to move. Only the operator can clear it."}
    full = f"{BRIDGE_URL}/{path.lstrip('/')}"
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(full, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    if BRIDGE_TOKEN:
        req.add_header("Authorization", f"Bearer {BRIDGE_TOKEN}")
    try:
        with urllib.request.urlopen(req, timeout=timeout or BRIDGE_TIMEOUT_S) as r:
            raw = r.read().decode("utf-8", errors="replace")
    except urllib.error.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            pass
        return {"error": f"bridge HTTP {e.code}", "husky": HUSKY_KEY, "detail": detail[:500]}
    except urllib.error.URLError as e:
        return {"error": f"bridge unreachable: {e.reason}", "husky": HUSKY_KEY}
    except Exception as e:
        return {"error": f"bridge call failed: {e}", "husky": HUSKY_KEY}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "non-JSON from bridge", "raw": raw[:500]}


def _bridge_reachable(timeout_s: float = 1.0) -> bool:
    try:
        with urllib.request.urlopen(BRIDGE_URL.rstrip("/") + "/state",
                                    timeout=timeout_s) as r:
            return 200 <= r.status < 300
    except Exception:
        return False


def _pose() -> Optional[Tuple[float, float, float]]:
    try:
        s = _bridge_post("get_robot_state", timeout=5)
        return float(s["x"]), float(s["y"]), float(s["yaw"])
    except Exception:
        return None


def _settle(timeout_s: float = 15.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if _bridge_post("get_robot_state", timeout=5).get("mode") == "idle":
                return
        except Exception:
            return
        time.sleep(0.15)


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _with_final_pose(result: Dict[str, Any]) -> Dict[str, Any]:
    """Settle, then stamp the tool result with the robot's REAL final pose.

    Without this the model narrates the position it remembers from before
    the move (observed live: "my current position is (3.00, 3.00)" right
    after physically driving to x=5.02). The settled pose in the result is
    the evidence the child agent must report from.
    """
    if isinstance(result, dict) and "error" not in result:
        _settle()
        p = _pose()
        if p is not None:
            result["final_pose"] = {"x": round(p[0], 3), "y": round(p[1], 3),
                                    "yaw_deg": round(math.degrees(p[2]), 1)}
            result["note_pose"] = "final_pose is the settled position after this motion; report THIS, not a remembered position"
    return result


def _motion_blocked() -> Optional[Dict[str, Any]]:
    """Preflight for every motion tool: unit e-stop first, then the
    bridge's own live fault field."""
    if _ESTOP.is_set():
        return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": HUSKY_KEY,
                "hint": "Motion is disabled by the operator e-stop. Nothing moved. "
                        "Report this instead of retrying."}
    state = _bridge_post("get_robot_state", timeout=2)
    fault = state.get("fault") if isinstance(state, dict) else None
    if fault:
        return {"error": "bridge_fault", "fault": fault, "husky": HUSKY_KEY,
                "hint": "The bridge reports a fault on this robot. Do not move it; "
                        "report the fault to the operator."}
    return None


# --- Activity ring ---------------------------------------------------

ACTIVITY = collections.deque(maxlen=20)
_ACTIVITY_LOCK = threading.Lock()


def _record_activity(tool: str, args: Dict[str, Any], result: Dict[str, Any]) -> None:
    if isinstance(result, dict) and "error" in result:
        summary = f"err: {result['error']}"
    else:
        bits = []
        for k in ("x", "y", "yaw", "mode", "accepted", "clamped_to", "arrived", "error_m"):
            if isinstance(result, dict) and k in result:
                v = result[k]
                bits.append(f"{k}={round(v, 3) if isinstance(v, float) else v}")
                if len(bits) >= 3:
                    break
        summary = ", ".join(bits) if bits else "ok"
    with _ACTIVITY_LOCK:
        ACTIVITY.append({"ts": _now_iso(), "tool": tool,
                         "args": {k: v for k, v in (args or {}).items()},
                         "summary": summary})


# --- Tools (husky id baked in; no husky parameter anywhere) -----------

def tool_get_status(**_: Any) -> Dict[str, Any]:
    """Pose + velocity + fault straight from this husky's bridge."""
    state = _bridge_post("get_robot_state", timeout=5)
    if not isinstance(state, dict):
        return {"error": "bad bridge response", "husky": HUSKY_KEY}
    if "error" in state:
        return state
    x, y = state.get("x"), state.get("y")
    try:
        pose_text = f"pose=({float(x):.3f}, {float(y):.3f})"
    except (TypeError, ValueError):
        pose_text = "pose=UNKNOWN"
    return {
        "husky": HUSKY_KEY,
        "x": x, "y": y, "yaw": state.get("yaw"),
        # A redundant, copy-safe field is deliberate. Small models sometimes
        # transpose signs while converting separate x/y fields into prose.
        # Downstream supervisors need a stable spatial contract, not arithmetic.
        "pose_text": pose_text,
        "response_contract": f"Copy exactly in your answer: {pose_text}",
        "v_linear": state.get("v_linear"), "v_angular": state.get("v_angular"),
        "mode": state.get("mode"), "fault": state.get("fault"),
        "estop": estop_state(),
    }


def tool_drive_forward(distance_m: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Straight-line motion along the current heading. Clamped to +-10 m."""
    blocked = _motion_blocked()
    if blocked is not None:
        return blocked
    d = float(distance_m)
    clamped = max(-DRIVE_CLAMP_M, min(DRIVE_CLAMP_M, d))
    result = _bridge_post("drive_forward", {"distance": clamped})
    if isinstance(result, dict) and clamped != d:
        result["clamped_to"] = clamped
        result["requested_m"] = d
        result["note"] = f"distance clamped to +-{DRIVE_CLAMP_M:g} m"
    return _with_final_pose(result)


def tool_turn(angle_deg: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Rotate in place. Positive = counter-clockwise. Clamped to +-180 deg."""
    blocked = _motion_blocked()
    if blocked is not None:
        return blocked
    a = float(angle_deg)
    clamped = max(-TURN_CLAMP_DEG, min(TURN_CLAMP_DEG, a))
    result = _bridge_post("turn", {"angle": math.radians(clamped)})
    if isinstance(result, dict) and clamped != a:
        result["clamped_to"] = clamped
        result["requested_deg"] = a
        result["note"] = f"angle clamped to +-{TURN_CLAMP_DEG:g} deg"
    return _with_final_pose(result)


def tool_drive_to_xy(x: float = 0.0, y: float = 0.0, tolerance_m: float = 0.15,
                     max_iters: int = 3, **_: Any) -> Dict[str, Any]:
    """Drive to an absolute (x, y). Closed-loop turn-then-drive, gain
    compensated -- same strategy as the coordinator's drive_to_xy.
    Refuses targets more than 30 m from the robot's current position."""
    blocked = _motion_blocked()
    if blocked is not None:
        return blocked
    x, y = float(x), float(y)
    start = _pose()
    if start is None:
        return {"error": "cannot read pose", "husky": HUSKY_KEY,
                "hint": "Refusing to drive blind. Check the bridge."}
    dist0 = math.hypot(x - start[0], y - start[1])
    if dist0 > DRIVE_TO_XY_MAX_M:
        return {"error": "target_too_far", "husky": HUSKY_KEY,
                "target_xy": [x, y], "distance_m": round(dist0, 2),
                "limit_m": DRIVE_TO_XY_MAX_M,
                "hint": f"REFUSED: the target is {dist0:.1f} m away, beyond the "
                        f"{DRIVE_TO_XY_MAX_M:g} m unit-agent limit. Report the "
                        f"limit to the operator; do not substitute a nearer point."}
    iters = 0
    end = start
    for _i in range(int(max_iters)):
        iters += 1
        p = _pose()
        if p is None:
            break
        end = p
        dx, dy = x - p[0], y - p[1]
        remaining = math.hypot(dx, dy)
        if remaining <= tolerance_m:
            break
        residual = _norm_angle(math.atan2(dy, dx) - p[2])
        if abs(residual) > math.radians(3.0):
            _bridge_post("turn", {"angle": residual / TURN_GAIN})
            _settle()
        p = _pose()
        if p is None:
            break
        remaining = math.hypot(x - p[0], y - p[1])
        _bridge_post("drive_forward", {"distance": remaining / DRIVE_GAIN})
        _settle()
    end = _pose() or end
    err = math.hypot(x - end[0], y - end[1])
    return {"husky": HUSKY_KEY, "target_xy": [round(x, 3), round(y, 3)],
            "final_xy": [round(end[0], 3), round(end[1], 3)],
            "error_m": round(err, 3), "iterations": iters,
            "arrived": err <= tolerance_m + 0.03,
            "start_xy": [round(start[0], 3), round(start[1], 3)]}


def tool_stop(**_: Any) -> Dict[str, Any]:
    """Halt this husky immediately. Always allowed, even under e-stop."""
    return _bridge_post("stop_robot", {}, timeout=4, _bypass_estop=True)


def tool_report(**_: Any) -> Dict[str, Any]:
    """One-paragraph self-report: live pose + the recent activity ring."""
    state = _bridge_post("get_robot_state", timeout=5)
    with _ACTIVITY_LOCK:
        recent = list(ACTIVITY)
    if isinstance(state, dict) and "x" in state:
        pose_txt = (f"at ({float(state['x']):.2f}, {float(state['y']):.2f}), "
                    f"yaw {float(state.get('yaw', 0.0)):.2f} rad, "
                    f"mode {state.get('mode', '?')}"
                    + (f", FAULT: {state['fault']}" if state.get("fault") else ""))
    else:
        pose_txt = "pose unreadable (bridge did not answer)"
    if recent:
        acts = "; ".join(f"{e['tool']} ({e['summary']})" for e in recent[-6:])
        act_txt = f"Recent actions (newest last): {acts}."
    else:
        act_txt = "No actions taken yet this session."
    estop_txt = (f" E-stop is ENGAGED ({_ESTOP_REASON})." if _ESTOP.is_set() else "")
    paragraph = (f"{AGENT_NAME} reporting on {HUSKY_KEY}: {pose_txt}.{estop_txt} "
                 f"{act_txt}")
    return {"husky": HUSKY_KEY, "report": paragraph,
            "recent_actions": recent, "estop": estop_state()}


TOOLS: Dict[str, Dict[str, Any]] = {
    "get_status": {
        "description": ("Read this robot's live state from its bridge: x, y, yaw, "
                        "v_linear, v_angular, mode, fault. Call before reporting "
                        "any position."),
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_get_status,
    },
    "drive_forward": {
        "description": ("Drive this robot straight along its current heading. "
                        "distance_m is clamped to +-10 m. Positive = forward."),
        "parameters": {"type": "object", "properties": {
            "distance_m": {"type": "number"},
        }, "required": ["distance_m"]},
        "impl": tool_drive_forward,
    },
    "turn": {
        "description": ("Rotate this robot in place. angle_deg is clamped to "
                        "+-180. Positive = counter-clockwise."),
        "parameters": {"type": "object", "properties": {
            "angle_deg": {"type": "number"},
        }, "required": ["angle_deg"]},
        "impl": tool_turn,
    },
    "drive_to_xy": {
        "description": ("Drive this robot to an absolute (x, y) coordinate. "
                        "Closed-loop: reads pose, turns, drives, corrects. "
                        "Refuses targets more than 30 m away."),
        "parameters": {"type": "object", "properties": {
            "x": {"type": "number"}, "y": {"type": "number"},
            "tolerance_m": {"type": "number"},
        }, "required": ["x", "y"]},
        "impl": tool_drive_to_xy,
    },
    "stop": {
        "description": "Halt this robot immediately.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_stop,
    },
    "report": {
        "description": ("One-paragraph self-report: live pose plus the last few "
                        "actions this agent took. Use when the foreman or the "
                        "operator asks how this robot is doing."),
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_report,
    },
}


def dispatch(tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    spec = TOOLS.get(tool_name)
    if spec is None:
        return {"error": f"unknown tool: {tool_name}",
                "known_tools": sorted(TOOLS.keys())}
    try:
        result = spec["impl"](**(args or {}))
    except TypeError as e:
        return {"error": f"bad args: {e}"}
    except Exception as e:
        detail = f"{type(e).__name__}: {e}"
        _log(f"tool {tool_name} failed: {detail}")
        traceback.print_exc()
        return {"error": "tool_execution_failed", "detail": detail[:300],
                "hint": "This tool did NOT run. Do not report its effect as done."}
    _record_activity(tool_name, args or {}, result)
    return result


# --- HTTP server (platform tool callback; mirrors swarm_agent) --------

def _short(d: Any, n: int = 120) -> str:
    s = json.dumps(d, default=str)
    return s if len(s) <= n else s[:n - 3] + "..."


def _make_handler() -> Any:
    # Same trusted-origin surface as the coordinator: OMNISIM_ALLOWED_ORIGINS
    # via trusted_origins_from_env, plus the swarm operator UI dev server.
    trusted_origins = trusted_origins_from_env(
        extra=("http://localhost:5174", "http://127.0.0.1:5174"))
    token = bearer_token()
    action_lock = threading.RLock()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_: Any) -> None: pass

        def _cors(self) -> None:
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers",
                             "Content-Type, Authorization, X-OmniSim-Token")

        def _guard(self) -> None:
            self._response_origin = checked_origin(self.headers, trusted_origins)
            require_authorized(self.headers, token)

        def _write_json(self, status: int, payload: Dict[str, Any]) -> None:
            body = json.dumps(payload, default=str).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self) -> None:
            try:
                self._guard()
                self.send_response(204); self._cors(); self.end_headers()
            except RequestError as exc:
                self._write_json(exc.status, error_envelope(exc.code, exc.message, exc.details))

        def do_GET(self) -> None:
            try:
                self._guard()
            except RequestError as exc:
                self._write_json(exc.status, error_envelope(exc.code, exc.message, exc.details))
                return
            if self.path == "/estop":
                self._write_json(200, estop_state()); return
            if self.path == "/health":
                self._write_json(200, {
                    "ok": True,
                    "agent": AGENT_NAME,
                    "husky": HUSKY_KEY,
                    "bridge_url": BRIDGE_URL,
                    "bridge_reachable": _bridge_reachable(),
                    "uptime_s": round(time.time() - _START_TS, 1),
                    "tool_count": len(TOOLS),
                    "estop": estop_state(),
                })
                return
            self.send_error(404)

        def do_POST(self) -> None:
            try:
                self._guard()
                if self.path == "/estop":
                    # Operator surface. Deliberately NOT a registered tool.
                    body = read_json(self, allow_empty=True) or {}
                    action = str(body.get("action") or "engage").strip().lower()
                    reason = str(body.get("reason") or "operator").strip()
                    out = estop_clear() if action == "clear" else estop_engage(reason)
                    self._write_json(200, out)
                    return
                if self.path != "/tool":
                    self._write_json(404, error_envelope("not_found", "Unknown callback path."))
                    return
                data = read_json(self, allow_empty=False)
            except RequestError as exc:
                self._write_json(exc.status, error_envelope(exc.code, exc.message, exc.details))
                return
            # Accept both {name, arguments} (platform delegation shape) and
            # the coordinator's flat {tool, ...args} shape.
            if "name" in data:
                tool_name = data.get("name", "")
                args = data.get("arguments") or {}
            else:
                tool_name = data.pop("tool", "")
                args = data
            if not isinstance(tool_name, str) or not tool_name.strip():
                self._write_json(400, error_envelope(
                    "missing_field", "Field 'name' (or 'tool') is required.",
                    {"field": "name"}))
                return
            if not isinstance(args, dict):
                args = {}
            _log(f"[TOOL] {tool_name}({_short(args)})")
            with action_lock:
                result = dispatch(tool_name, args)
            self._write_json(200, {"status": "ok", "tool": tool_name, "result": result})

    return Handler


def start_tool_server(port: int) -> int:
    token = bearer_token()
    validate_bind_host(UNIT_HOST, token)
    try:
        server = http.server.ThreadingHTTPServer((UNIT_HOST, port), _make_handler())
    except OSError as e:
        _log(f"[WARN] port {port} taken ({e}); falling back to random.")
        server = http.server.ThreadingHTTPServer((UNIT_HOST, 0), _make_handler())
    bound = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return bound


# --- Profile push -----------------------------------------------------

def build_profile_settings(tool_callback_url: str) -> Dict[str, Any]:
    persona = (
        f"You are {AGENT_NAME}, the dedicated unit agent for robot {HUSKY_KEY}. "
        f"You are responsible ONLY for this one robot: its status, movement, and "
        f"safety. Answer precisely about your robot; refuse tasks about other "
        f"huskies (your foreman HuskySwarm handles the fleet)."
    )
    main_task = (
        f"You are {AGENT_NAME}, the unit agent for exactly one Clearpath Husky "
        f"({HUSKY_KEY}) in OmniSim's omnilink_husky_swarm.omniworld world. Your tools "
        f"are already scoped to your robot -- there is no robot parameter, and "
        f"you cannot address any other robot.\n\n"
        f"TOOLS\n"
        f"  get_status                 live x, y, yaw, velocity, mode, fault\n"
        f"  drive_forward(distance_m)  straight-line motion, clamped to +-10 m\n"
        f"  turn(angle_deg)            in-place rotation, clamped to +-180 deg\n"
        f"  drive_to_xy(x, y)          closed-loop go-to-point; refuses targets\n"
        f"                             more than 30 m away\n"
        f"  stop                       immediate halt (always allowed)\n"
        f"  report                     one-paragraph self-report: pose + recent actions\n\n"
        f"RULES\n"
        f"1. EVERY request starts with a live tool call. For status, health, "
        f"location, or a generic check-in, call get_status before writing any "
        f"text. A reply without a tool call is invalid.\n"
        f"2. If a request concerns a different robot or the whole fleet, refuse "
        f"and point at your foreman HuskySwarm.\n"
        f"3. If a motion tool is refused (e-stop, fault, out-of-range target), "
        f"report the refusal and its reason; do not retry or substitute a "
        f"smaller version of the order.\n"
        f"4. On 'stop' / 'halt' / 'freeze': call stop first, then reply.\n"
        f"5. Keep replies short and operational; numbers first.\n"
        f"6. STATUS RESPONSE CONTRACT. Copy the get_status result's `pose_text` "
        f"field character-for-character into the answer. Never reconstruct, "
        f"transpose, or change its signs. If it says `pose=UNKNOWN`, report an "
        f"unreadable pose rather than guessing."
    )
    tool_defs = [
        {"name": n, "description": TOOLS[n]["description"], "parameters": TOOLS[n]["parameters"]}
        for n in TOOLS
    ]
    # Same convention as the coordinator: the platform reads mainTask as the
    # system instruction, so the persona is baked in front of it.
    return {
        "agentName": AGENT_NAME,
        "persona": persona,
        "mainTask": persona + "\n\n" + main_task,
        "engine": "g1-engine",
        "temperature": 0.15,
        "availableTools": ", ".join(t["name"] for t in tool_defs),
        "availableToolDetails": tool_defs,
        "allowToolUse": True,
        "toolCallbackUrl": tool_callback_url,
        "delegationTimeout": 90,
    }


# --- Presence heartbeat (adapted copy of swarm_agent's) ---------------

def start_presence_heartbeat(agent_name: str, omni_key: str,
                             engine: str, endpoint: str = "") -> threading.Event:
    stop = threading.Event()

    def _post_presence(body: Dict[str, Any]) -> None:
        req = urllib.request.Request(
            f"{BASE_URL.rstrip('/')}/api/relay-heartbeat",
            data=json.dumps(body).encode("utf-8"),
            method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {omni_key}"},
        )
        with urllib.request.urlopen(req, timeout=8):
            pass

    def _loop() -> None:
        detail: Dict[str, Any] = {
            "tools": len(TOOLS),
            "engine": engine,
            "husky": HUSKY_KEY,
            "interval_ms": int(PRESENCE_INTERVAL_S * 1000),
        }
        while not stop.is_set():
            try:
                body: Dict[str, Any] = {
                    "agentName": agent_name,
                    "kind": "agent",
                    "detail": dict(detail),
                }
                if endpoint:
                    body["endpoint"] = endpoint
                _post_presence(body)
            except Exception:
                pass  # presence is telemetry; a missed beat reads as offline
            stop.wait(PRESENCE_INTERVAL_S)

    threading.Thread(target=_loop, name="omnilink-presence", daemon=True).start()
    return stop


# --- Self-test (no network; bridge stubbed) ---------------------------

def self_test() -> int:
    """Python-only sanity: every advertised tool dispatches against a stub
    bridge; guards fire; the profile payload has the right shape; the
    coordinator's team derivation works on a fake 4-husky bridges env."""
    global _bridge_post
    failures: List[str] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        status = "PASS" if ok else "FAIL"
        _log(f"  [{status}] {name}" + (f" -- {detail}" if detail and not ok else ""))
        if not ok:
            failures.append(name)

    # Stub the bridge.
    calls: List[Tuple[str, Dict[str, Any]]] = []
    state = {"x": 0.1, "y": 0.2, "yaw": 0.0, "v_linear": 0.0, "v_angular": 0.0,
             "mode": "idle", "fault": None}

    real_bridge_post = _bridge_post

    def stub_bridge_post(path, payload=None, timeout=None, _bypass_estop=False):
        p = path.lstrip("/")
        if (not _bypass_estop) and _ESTOP.is_set() and p in MOTION_PATHS:
            return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": HUSKY_KEY}
        calls.append((p, dict(payload or {})))
        if p == "get_robot_state":
            return dict(state)
        return {"accepted": True, "path": p}

    _bridge_post = stub_bridge_post
    try:
        # 1. Every advertised tool dispatches without error.
        happy_args: Dict[str, Dict[str, Any]] = {
            "get_status": {},
            "drive_forward": {"distance_m": 1.5},
            "turn": {"angle_deg": 45},
            "drive_to_xy": {"x": 1.0, "y": 1.0},
            "stop": {},
            "report": {},
        }
        check("tool table covers happy_args exactly",
              set(happy_args) == set(TOOLS),
              f"tools={sorted(TOOLS)} args={sorted(happy_args)}")
        for name in TOOLS:
            res = dispatch(name, happy_args.get(name, {}))
            check(f"tool {name} dispatches", isinstance(res, dict) and "error" not in res,
                  json.dumps(res, default=str)[:200])
        status_result = dispatch("get_status", {})
        check("get_status supplies copy-safe pose contract",
              status_result.get("pose_text") == "pose=(0.100, 0.200)"
              and status_result.get("response_contract")
              == "Copy exactly in your answer: pose=(0.100, 0.200)",
              json.dumps(status_result, default=str)[:240])

        # 2. Clamps. Motion tools settle + re-read pose after the move
        # (_with_final_pose), so the motion call is no longer calls[-1] —
        # search backwards for the last one of the right kind.
        def last_call(kind):
            for c in reversed(calls):
                if c[0] == kind:
                    return c
            return None

        res = dispatch("drive_forward", {"distance_m": 50})
        check("drive_forward clamps to +-10",
              res.get("clamped_to") == DRIVE_CLAMP_M
              and last_call("drive_forward") == ("drive_forward", {"distance": DRIVE_CLAMP_M}),
              json.dumps(res, default=str)[:200])
        check("drive_forward reports settled final_pose",
              isinstance(res.get("final_pose"), dict) and "x" in res["final_pose"],
              json.dumps(res.get("final_pose", {}), default=str)[:120])
        res = dispatch("turn", {"angle_deg": -720})
        check("turn clamps to +-180",
              res.get("clamped_to") == -TURN_CLAMP_DEG
              and abs(last_call("turn")[1]["angle"] + math.pi) < 1e-9,
              json.dumps(res, default=str)[:200])

        # 3. drive_to_xy distance guard.
        res = dispatch("drive_to_xy", {"x": 100.0, "y": 0.0})
        check("drive_to_xy refuses >30 m targets",
              res.get("error") == "target_too_far",
              json.dumps(res, default=str)[:200])

        # 4. E-stop gates motion, stop still works, clear restores.
        estop_engage("self-test")
        res = dispatch("drive_forward", {"distance_m": 1})
        check("estop refuses drive_forward", res.get("error") == "estop_engaged",
              json.dumps(res, default=str)[:200])
        res = dispatch("stop", {})
        check("stop bypasses estop", isinstance(res, dict) and "error" not in res,
              json.dumps(res, default=str)[:200])
        estop_clear()
        res = dispatch("drive_forward", {"distance_m": 1})
        check("motion restored after estop clear",
              isinstance(res, dict) and "error" not in res,
              json.dumps(res, default=str)[:200])

        # 5. Bridge fault preflight refuses motion.
        state["fault"] = "wheel_stall"
        res = dispatch("turn", {"angle_deg": 10})
        check("bridge fault refuses motion", res.get("error") == "bridge_fault",
              json.dumps(res, default=str)[:200])
        state["fault"] = None

        # 6. Unknown tool is a clean error.
        res = dispatch("no_such_tool", {})
        check("unknown tool errors cleanly", "unknown tool" in str(res.get("error", "")),
              json.dumps(res, default=str)[:200])

        # 7. report mentions the pose and recent actions.
        res = dispatch("report", {})
        check("report is a paragraph with pose",
              AGENT_NAME in res.get("report", "") and "0.10" in res.get("report", ""),
              json.dumps(res, default=str)[:300])

        # 8. Profile payload shape.
        settings = build_profile_settings(f"http://127.0.0.1:51521/tool")
        detail_names = [t["name"] for t in settings["availableToolDetails"]]
        check("profile advertises exactly the tool table",
              detail_names == list(TOOLS)
              and settings["availableTools"] == ", ".join(TOOLS),
              json.dumps(detail_names))
        check("profile has callback + delegationTimeout",
              settings["toolCallbackUrl"].endswith("/tool")
              and settings["delegationTimeout"] == 90
              and settings["allowToolUse"] is True)
        check("profile persona names this husky",
              AGENT_NAME in settings["persona"] and HUSKY_KEY in settings["persona"]
              and settings["mainTask"].startswith(settings["persona"]))
        for t in settings["availableToolDetails"]:
            props = (t["parameters"] or {}).get("properties", {})
            check(f"tool schema {t['name']} has no husky parameter",
                  "husky" not in props, json.dumps(props))

        # 9. Coordinator team derivation from a fake 4-husky bridges env.
        import swarm_agent as _swarm
        fake = {"husky_ne": "http://x:1", "husky_nw": "http://x:2",
                "husky_se": "http://x:3", "husky_sw": "http://x:4"}
        team = _swarm._team_agent_names(fake)
        check("swarm team derivation",
              team == ["HuskyNE", "HuskyNW", "HuskySE", "HuskySW"], json.dumps(team))
    finally:
        _bridge_post = real_bridge_post

    _log(f"self-test: {len(failures)} failure(s)"
         + (f": {failures}" if failures else " -- all checks passed"))
    return 1 if failures else 0


# --- Main -------------------------------------------------------------

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="husky_unit_agent",
        description="Dedicated OmniLink unit agent for one Husky of the swarm world.")
    parser.add_argument("--husky", required=True,
                        help="Husky id: ne, nw, se, sw (or husky_ne, ...).")
    parser.add_argument("--port", type=int, default=None,
                        help="Tool-callback port (default: 51521-51524 by husky id).")
    parser.add_argument("--self-test", action="store_true",
                        help="Run the offline sanity checks (bridge stubbed) and exit.")
    args = parser.parse_args(argv)

    port = _configure(args.husky, args.port)

    if args.self_test:
        return self_test()

    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        _log("ERROR: OMNI_KEY not set. Get one at https://www.omnilink-agents.com/key"
             " or run: python -m omnisim key")
        return 1

    from omnilink.client import OmniLinkClient  # noqa: E402 (needs omnilink-lib)

    bound = start_tool_server(port)
    tool_callback = f"http://127.0.0.1:{bound}/tool"
    _log(f"unit agent for {HUSKY_KEY} (bridge {BRIDGE_URL})")
    _log(f"tool callback: {tool_callback}")
    _log(f"{len(TOOLS)} tools registered; e-stop: POST /estop (operator-only)")
    if not BRIDGE_TOKEN:
        _log("[SECURITY] bridge auth OFF (OMNISIM_BRIDGE_TOKEN unset)")

    client = OmniLinkClient(omni_key=omni_key, base_url=BASE_URL, timeout=30)
    settings = build_profile_settings(tool_callback)
    ensure_profile(client, AGENT_NAME, settings)

    presence_stop = start_presence_heartbeat(
        AGENT_NAME, omni_key,
        engine=str(settings.get("engine", "")),
        endpoint=tool_callback,
    )
    _log(f"presence heartbeat: every {PRESENCE_INTERVAL_S:g}s -> "
         f"{BASE_URL.rstrip('/')}/api/relay-heartbeat")
    _log("ready.")
    try:
        while True:
            time.sleep(3.0)
    except KeyboardInterrupt:
        _log("shutting down.")
        presence_stop.set()
        return 0


if __name__ == "__main__":
    sys.exit(main())
