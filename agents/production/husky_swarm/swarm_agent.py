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

"""HuskySwarm -- OmniLink-driven swarm coordinator for OmniSim's
4-Husky world.

The agent owns:

  - A rich PRIMITIVE surface (bridge-level commands the LLM composes
    into multi-step behaviour at runtime).
  - PARALLEL EXECUTION across the swarm in a single tool call
    (execute_parallel).
  - PERSISTENT MEMORY: named waypoints, recorded routines, and free-form
    key/value swarm facts, stored in a local SQLite index. Carries
    across sessions, the world reload, even an OmniSim restart.
  - OBSERVABILITY: a structured activity log + per-Husky metrics
    (distance travelled, mode time, fault counts).
  - META-TOOLS: find_tools, invoke_tool, list_tools (meta-tool pattern)
    so the agent can introspect and discover its own surface.
  - SAFETY: tool-loop budget (no more than 12 tool calls in 30s) and
    arena-bounds guard (refuse motion that would push a Husky beyond
    +-7 m).

Run:

    export OMNI_KEY="olink_..."
    pip install omnilink
    python agents/templates/husky_swarm/swarm_agent.py

Then open https://omnilink-agents.com, pick "HuskySwarm" in the
profile dropdown, and chat.
"""

from __future__ import annotations

import http.server
import json
import math
import os
import re
import sqlite3
import subprocess
import sys
import threading
import time
import traceback
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

_REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_REPO_ROOT / "packages" / "omnisim-bridges" / "src"))
sys.path.insert(0, str(_REPO_ROOT / "agents" / "production"))
from _lib.supervision import (  # noqa: E402
    execute_verified_sequence,
    failure_error,
    find_duplicate_resource_claims,
)
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

try:
    from omnilink.client import OmniLinkClient  # type: ignore
except ImportError:
    print("ERROR: omnilink-lib is not installed.\n    pip install omnilink")
    sys.exit(1)


# ─── Configuration ──────────────────────────────────────────────────

THIS_DIR = Path(__file__).resolve().parent
PROFILE_PATH = THIS_DIR / "profile.json"
BASE_URL = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com")

SWARM_PORT = int(os.environ.get("HUSKY_SWARM_PORT", "51520"))
SWARM_HOST = os.environ.get("HUSKY_SWARM_HOST", "127.0.0.1").strip() or "127.0.0.1"
POLL_INTERVAL_S = float(os.environ.get("HUSKY_SWARM_POLL_INTERVAL_S", "3"))
BRIDGE_TIMEOUT_S = float(os.environ.get("HUSKY_SWARM_BRIDGE_TIMEOUT_S", "10"))
# Ceiling for one execute_parallel batch. Compound motions run much longer
# than a single bridge call, so this is not BRIDGE_TIMEOUT_S.
PARALLEL_JOIN_TIMEOUT_S = float(os.environ.get("HUSKY_SWARM_PARALLEL_TIMEOUT_S", "180"))

# Arena bounds for the husky_swarm world. Used by the safety guard.
ARENA_BOUND_M = float(os.environ.get("HUSKY_SWARM_ARENA_BOUND_M", "7.0"))

# Tool-loop budget: bound run-away tool firing patterns. Same idea as
# a rate-limit gate. Window is a rolling 30 seconds.
TOOL_BUDGET_WINDOW_S = 30.0
TOOL_BUDGET_TOTAL = int(os.environ.get("HUSKY_SWARM_TOOL_BUDGET", "30"))
TOOL_BUDGET_PER_TOOL = int(os.environ.get("HUSKY_SWARM_TOOL_BUDGET_PER", "14"))

# Persistent store. Override location via HUSKY_SWARM_DB.
DB_PATH = Path(
    os.environ.get("HUSKY_SWARM_DB")
    or (Path.home() / ".omnisim" / "husky_swarm.sqlite3")
).expanduser()


def _parse_bridges_env() -> Dict[str, str]:
    spec = os.environ.get("HUSKY_SWARM_BRIDGES", "").strip()
    if not spec:
        # Dedicated port block. Other demos on the 8765-8767 port block use
        # the shared repo-wide default, so the swarm claimed ports it had to
        # race for -- losing the race silently handed the agent some other
        # robot it would drive as a Husky.
        return {
            "husky_ne": "http://127.0.0.1:8865",
            "husky_nw": "http://127.0.0.1:8866",
            "husky_se": "http://127.0.0.1:8867",
            "husky_sw": "http://127.0.0.1:8868",
        }
    out: Dict[str, str] = {}
    for piece in spec.split(","):
        piece = piece.strip()
        if not piece or "=" not in piece:
            continue
        k, v = piece.split("=", 1)
        out[k.strip()] = v.strip().rstrip("/")
    return out


BRIDGES = _parse_bridges_env()
SWARM_QUADRANT_SPAWN = {
    "husky_ne": (3.0,  3.0),
    "husky_nw": (-3.0, 3.0),
    "husky_se": (3.0, -3.0),
    "husky_sw": (-3.0, -3.0),
}


# ─── HTTP plumbing ──────────────────────────────────────────────────

# ─── Safety interlock ───────────────────────────────────────────────
#
# Everything here is enforced OUTSIDE the model's reach. The LLM cannot call
# it, cannot see it in its tool list, and cannot argue with it. That matters
# more than which model is driving: benchmarked 2026-07-22, llama3.1:8b
# responded to a QUESTION ("confirm the robots already advanced 5 m") by
# driving them 4.67 m. A local model is not a safer model -- this layer is
# what makes any model safe, and it is identical for every engine.

_ESTOP = threading.Event()
_ESTOP_REASON = ""

# Which client currently owns motion. Each Husky also has an omnilink_chat
# window that writes the same MobileBridge.motion state, so without this a
# stray chat command mid-execute_parallel just wins, silently.
_CONTROL_OWNER = ""
_CONTROL_LOCK = threading.Lock()
CONTROL_OWNER_REQUIRED = os.environ.get("HUSKY_SWARM_REQUIRE_OWNER", "").strip() == "1"

BRIDGE_TOKEN = os.environ.get("OMNISIM_BRIDGE_TOKEN", "").strip()

# Mirrors VELOCITY_MAX_S in omnilink_mobile_bridge: how long a commanded
# twist keeps running before the bridge expires it. Used to bound the
# worst-case travel of a set_velocity call.
VELOCITY_EXPIRY_S = float(os.environ.get("HUSKY_SWARM_VELOCITY_EXPIRY_S", "12"))

# Every path that can put a robot in motion. Membership here is what subjects
# a tool to the e-stop; keep it in sync when adding motion tools.
MOTION_PATHS = {"drive_forward", "turn", "set_velocity", "reset_to_home"}


def estop_engage(reason: str = "operator") -> Dict[str, Any]:
    """Latch the e-stop and halt every robot. Not a registered tool."""
    global _ESTOP_REASON
    _ESTOP_REASON = reason
    _ESTOP.set()
    halted = []
    for husky in BRIDGES:
        try:
            _bridge_post(husky, "stop_robot", {}, timeout=4, _bypass_estop=True)
            halted.append(husky)
        except Exception:
            pass
    return {"estop": "engaged", "reason": reason, "halted": halted}


def estop_clear() -> Dict[str, Any]:
    global _ESTOP_REASON
    _ESTOP.clear()
    _ESTOP_REASON = ""
    return {"estop": "cleared"}


def estop_state() -> Dict[str, Any]:
    return {"engaged": _ESTOP.is_set(), "reason": _ESTOP_REASON}


def _bridge_post(husky: str, path: str, payload: Optional[Dict[str, Any]] = None,
                 timeout: Optional[float] = None,
                 _bypass_estop: bool = False) -> Dict[str, Any]:
    url = BRIDGES.get(husky)
    if not url:
        return {"error": f"unknown husky {husky!r}", "known_huskies": sorted(BRIDGES.keys())}
    # Motion is refused while the e-stop is latched. Telemetry reads still
    # work, so the agent can explain the situation instead of going blind.
    if (not _bypass_estop) and _ESTOP.is_set() and path.lstrip("/") in MOTION_PATHS:
        return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": husky,
                "hint": "Motion is disabled by the operator e-stop. Report this "
                        "and stop trying to move. Only the operator can clear it."}
    full = f"{url}/{path.lstrip('/')}"
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
        return {"error": f"bridge HTTP {e.code}", "husky": husky, "detail": detail[:500]}
    except urllib.error.URLError as e:
        return {"error": f"bridge unreachable: {e.reason}", "husky": husky}
    except Exception as e:
        return {"error": f"bridge call failed: {e}", "husky": husky}
    try:
        return json.loads(raw) if raw else {}
    except json.JSONDecodeError:
        return {"error": "non-JSON from bridge", "raw": raw[:500]}


# ─── Persistent store ───────────────────────────────────────────────
#
# Single SQLite file, three tables:
#
#   waypoints (name TEXT PK, x REAL, y REAL, note TEXT, updated_at TEXT)
#   routines  (name TEXT PK, steps_json TEXT, note TEXT, updated_at TEXT)
#   memories  (key TEXT PK, value TEXT, tags TEXT, updated_at TEXT)
#
# Persistent across sessions. The agent can build up a vocabulary of
# named locations, learned routines, and operator preferences over
# time. This is what turns the swarm from "follow my command" into
# "I remember last time you asked for sweep, it took 18 seconds".

_STORE_LOCK = threading.Lock()


def _store_init() -> None:
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    with _STORE_LOCK:
        with sqlite3.connect(str(DB_PATH)) as cx:
            cx.executescript("""
                CREATE TABLE IF NOT EXISTS waypoints (
                    name TEXT PRIMARY KEY,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    note TEXT,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS routines (
                    name TEXT PRIMARY KEY,
                    steps_json TEXT NOT NULL,
                    note TEXT,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS memories (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    tags TEXT,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS outcomes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    task TEXT NOT NULL,
                    success INTEGER NOT NULL,
                    notes TEXT,
                    elapsed_s REAL,
                    recorded_at TEXT NOT NULL
                );
            """)


def _store_exec(query: str, params: Tuple[Any, ...] = ()) -> List[Any]:
    with _STORE_LOCK:
        with sqlite3.connect(str(DB_PATH)) as cx:
            cx.row_factory = sqlite3.Row
            cur = cx.execute(query, params)
            return [dict(row) for row in cur.fetchall()]


def _store_write(query: str, params: Tuple[Any, ...] = ()) -> None:
    with _STORE_LOCK:
        with sqlite3.connect(str(DB_PATH)) as cx:
            cx.execute(query, params)
            cx.commit()


def _now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ─── Activity log + metrics (in-memory + occasional persistence) ───
#
# Activity log is a rolling N-entry list. Metrics are derived from
# bridge state polls + tool calls.

ACTIVITY_LOG: List[Dict[str, Any]] = []
ACTIVITY_LOG_MAX = 500
_METRICS: Dict[str, Dict[str, float]] = {h: {"distance_m": 0.0, "halts": 0, "tool_calls": 0} for h in BRIDGES}
_METRICS_LAST_POSE: Dict[str, Tuple[float, float]] = {}
_METRICS_LOCK = threading.Lock()


def _log_activity(tool: str, args: Dict[str, Any], result: Dict[str, Any]) -> None:
    ACTIVITY_LOG.append({
        "ts": _now_iso(),
        "tool": tool,
        "args": args,
        "result_summary": _summarize_result(result),
    })
    if len(ACTIVITY_LOG) > ACTIVITY_LOG_MAX:
        del ACTIVITY_LOG[:len(ACTIVITY_LOG) - ACTIVITY_LOG_MAX]


def _summarize_result(result: Dict[str, Any]) -> str:
    if not isinstance(result, dict):
        return str(result)[:120]
    if "error" in result:
        return f"err: {result['error']}"
    keys = []
    for k in ("accepted", "halted_at", "id", "x", "y", "yaw", "mode", "idle",
              "waited_s", "elapsed_s", "found", "count", "name", "tool"):
        if k in result:
            v = result[k]
            if isinstance(v, float):
                v = round(v, 3)
            elif isinstance(v, list) and len(v) > 3:
                v = f"[{len(v)} items]"
            keys.append(f"{k}={v}")
            if len(keys) >= 4:
                break
    return ", ".join(keys) if keys else "ok"


def _update_metrics_from_state(state: Dict[str, Dict[str, Any]]) -> None:
    """Called when get_swarm_state is invoked. Accumulates distance per
    Husky from successive pose reads."""
    with _METRICS_LOCK:
        for husky, st in state.items():
            if not isinstance(st, dict) or "x" not in st or "y" not in st:
                continue
            x, y = float(st["x"]), float(st["y"])
            prev = _METRICS_LAST_POSE.get(husky)
            if prev is not None:
                dx = x - prev[0]
                dy = y - prev[1]
                _METRICS[husky]["distance_m"] += math.hypot(dx, dy)
            _METRICS_LAST_POSE[husky] = (x, y)


# ─── Tool-loop budget ───────────────────────────────────────────────

_TOOL_HISTORY: List[Tuple[float, str]] = []
_TOOL_HISTORY_LOCK = threading.Lock()


def _check_tool_loop_budget(tool_name: str) -> Optional[Dict[str, Any]]:
    """Return refusal dict if the agent has fired too many tools in
    the rolling window. Mirrors the meta-tool pattern. Meta + observation
    tools are exempt so the agent can always introspect its way out
    of a loop."""
    if tool_name in {"find_tools", "list_tools", "get_activity_log",
                     "get_swarm_state", "get_swarm_metrics", "halt_all"}:
        return None
    now = time.time()
    cutoff = now - TOOL_BUDGET_WINDOW_S
    with _TOOL_HISTORY_LOCK:
        _TOOL_HISTORY[:] = [(ts, n) for ts, n in _TOOL_HISTORY if ts > cutoff]
        total = len(_TOOL_HISTORY)
        same = sum(1 for ts, n in _TOOL_HISTORY if n == tool_name)
        if total >= TOOL_BUDGET_TOTAL:
            return {
                "error": "tool_loop_budget_exceeded",
                "calls_in_window": total,
                "window_seconds": int(TOOL_BUDGET_WINDOW_S),
                "hint": (
                    f"You've made {total} tool calls in the last "
                    f"{int(TOOL_BUDGET_WINDOW_S)}s. STOP firing tools and "
                    "synthesize. Reply with TEXT: name what you did, name "
                    "what's left, and ask the operator whether to keep "
                    "going. Tools will work again once you produce a "
                    "text response and the operator speaks."
                ),
            }
        if same >= TOOL_BUDGET_PER_TOOL:
            return {
                "error": "same_tool_loop_exceeded",
                "tool": tool_name,
                "calls_in_window": same,
                "hint": (
                    f"You've called {tool_name} {same} times in the last "
                    f"{int(TOOL_BUDGET_WINDOW_S)}s. Try a different tool "
                    "or synthesize a text reply."
                ),
            }
    return None


def _record_tool_call(tool_name: str) -> None:
    with _TOOL_HISTORY_LOCK:
        _TOOL_HISTORY.append((time.time(), tool_name))


# ─── Arena bounds guard ─────────────────────────────────────────────

def _guard_drive_distance(husky: str, distance_m: float) -> Optional[Dict[str, Any]]:
    """Reject motion that would push the Husky beyond the arena bound.

    FAILS CLOSED. This used to swallow a bad telemetry read two ways -- an
    `except: return None` that let the motion through, and a
    `state.get("x", SPAWN[husky])` default that silently guarded the SPAWN
    pose instead of the real one. `_bridge_post` RETURNS an error dict rather
    than raising, so a timed-out read took the second path: a husky actually
    at (6.5, 3.0) was guarded as if it were at its (3.0, 3.0) spawn, and a
    command projecting to x = 10.1 m passed a 7.0 m bound. For
    set_husky_velocity this guard is the ONLY bound check there is.

    A guard that cannot see the robot must refuse, not assume.
    """
    state = _bridge_post(husky, "get_robot_state", timeout=2)
    reason = None
    if not isinstance(state, dict):
        reason = "telemetry read returned a non-object"
    elif failure_error(state):
        reason = f"telemetry read failed: {failure_error(state)}"
    else:
        missing = [k for k in ("x", "y", "yaw") if not isinstance(
            state.get(k), (int, float)) or isinstance(state.get(k), bool)]
        if missing:
            reason = f"telemetry read is missing {missing}"
    if reason is not None:
        return {
            "error": "pose_unavailable",
            "husky": husky,
            "detail": reason,
            "state": state if isinstance(state, dict) else None,
            "requested_distance_m": float(distance_m),
            "arena_bound_m": ARENA_BOUND_M,
            "hint": (
                f"REFUSED: nothing was commanded. The arena guard could not "
                f"read {husky}'s pose, and it is the only check standing "
                f"between this command and the +/-{ARENA_BOUND_M} m bound. "
                f"Do NOT retry blindly -- call get_swarm_state, confirm the "
                f"bridge is answering, and report the outage to the operator."
            ),
        }
    x = float(state["x"])
    y = float(state["y"])
    yaw = float(state["yaw"])
    nx = x + math.cos(yaw) * distance_m
    ny = y + math.sin(yaw) * distance_m
    if abs(nx) > ARENA_BOUND_M or abs(ny) > ARENA_BOUND_M:
        return {
            "error": "out_of_arena",
            "husky": husky,
            "current_xy": [round(x, 2), round(y, 2)],
            "projected_xy": [round(nx, 2), round(ny, 2)],
            "arena_bound_m": ARENA_BOUND_M,
            "hint": (
                f"REFUSED: {distance_m} m would put {husky} outside the "
                f"+/-{ARENA_BOUND_M} m arena. Report this limit to the operator "
                f"and ask what they want instead. Do NOT silently substitute a "
                f"shorter distance -- partially obeying an impossible order is "
                f"worse than refusing it, because the operator believes their "
                f"instruction was carried out."
            ),
        }
    return None


# ═══════════════════════════════════════════════════════════════════════
#                                  TOOLS
# ═══════════════════════════════════════════════════════════════════════
# Every tool returns a dict. `{"error": "..."}` for failures; never
# raise to the HTTP handler.


# ─── Bridge primitives ──────────────────────────────────────────────

def tool_list_huskies(**_: Any) -> Dict[str, Any]:
    """Probe every Husky bridge. Returns per-Husky online status + id."""
    out: List[Dict[str, Any]] = []
    for husky, url in sorted(BRIDGES.items()):
        resp = _bridge_post(husky, "list_robots", timeout=2)
        if "error" in resp:
            out.append({"husky": husky, "url": url, "online": False, "error": resp["error"]})
        else:
            entries = resp if isinstance(resp, list) else []
            first = entries[0] if entries else {}
            out.append({"husky": husky, "url": url, "online": True,
                        "id": first.get("id"), "model": first.get("model")})
    return {"huskies": out}


def tool_get_swarm_state(**_: Any) -> Dict[str, Any]:
    """Pose + mode + velocity for ALL 4 Huskies in one call."""
    out: Dict[str, Any] = {}
    for h in sorted(BRIDGES.keys()):
        out[h] = _bridge_post(h, "get_robot_state", timeout=2)
    _update_metrics_from_state(out)
    return {"state": out, "timestamp": time.time()}


def tool_get_husky_status(husky: str = "", **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
    state = _bridge_post(husky, "get_robot_state")
    if isinstance(state, dict) and "x" in state and "y" in state:
        _update_metrics_from_state({husky: state})
    return state


def tool_delegate_to_husky(husky: str = "", task: str = "", **_: Any) -> Dict[str, Any]:
    """Forward a natural-language task to one Husky's /prompt endpoint."""
    if not husky:
        return {"error": "husky is required"}
    if not task:
        return {"error": "task is required"}
    return _bridge_post(husky, "prompt", {"text": task}, timeout=60)


def tool_drive_husky(husky: str = "", distance_m: float = 0.0,
                     speed_m_s: Optional[float] = None, **_: Any) -> Dict[str, Any]:
    """Straight-line motion in metres along current heading.

    The bridge acknowledges motion before the model can safely describe the
    outcome.  Turn that asynchronous acknowledgement into a verified action:
    wait for idle, read the final pose, and return the measured displacement.
    This removes a reasoning round and, more importantly, keeps ``accepted``
    from being mistaken for ``completed``.
    """
    if not husky:
        return {"error": "husky is required"}
    blocked = _guard_drive_distance(husky, float(distance_m))
    if blocked is not None:
        return blocked
    if not _settle(husky, timeout_s=30.0):
        return {
            "error": "robot_not_idle",
            "husky": husky,
            "completed": False,
            "hint": "A previous bridge motion is still active. Nothing new was "
                    "commanded; stop or wait for that motion before retrying.",
        }
    start = _pose(husky)
    if start is None:
        return {"error": "cannot read pose", "husky": husky,
                "hint": "Nothing was commanded. Restore bridge telemetry first."}
    payload: Dict[str, Any] = {"distance": float(distance_m)}
    if speed_m_s is not None:
        payload["speed"] = float(speed_m_s)
    accepted = _bridge_post(husky, "drive_forward", payload)
    if not isinstance(accepted, dict):
        return {"error": "invalid bridge response", "husky": husky}
    if "error" in accepted:
        return accepted
    nominal_speed = abs(float(speed_m_s)) if speed_m_s is not None else 0.25
    settle_timeout = max(
        15.0, min(60.0, abs(float(distance_m)) / max(nominal_speed, 0.1) + 8.0))
    settled = _settle(husky, timeout_s=settle_timeout)
    if not settled:
        _bridge_post(husky, "stop_robot", _bypass_estop=True)
        _settle(husky, timeout_s=8.0)
    end = _pose(husky)
    if end is None:
        return {
            **accepted,
            "husky": husky,
            "completed": False,
            "error": "motion accepted but final pose is unreadable",
            "hint": "Do not claim completion; telemetry verification failed.",
        }
    dx, dy = end[0] - start[0], end[1] - start[1]
    moved = math.hypot(dx, dy)
    signed_moved = dx * math.cos(start[2]) + dy * math.sin(start[2])
    lateral_error = abs(-dx * math.sin(start[2]) + dy * math.cos(start[2]))
    target_error = abs(float(distance_m) - signed_moved)
    tolerance = max(0.08, abs(float(distance_m)) * 0.15)
    completed = settled and target_error <= tolerance and lateral_error <= tolerance
    return {
        **accepted,
        "husky": husky,
        "completed": completed,
        "settled": settled,
        "start_pose": {"x": round(start[0], 3), "y": round(start[1], 3),
                       "yaw": round(start[2], 4)},
        "final_pose": {"x": round(end[0], 3), "y": round(end[1], 3),
                       "yaw": round(end[2], 4)},
        "distance_achieved_m": round(moved, 3),
        "signed_distance_achieved_m": round(signed_moved, 3),
        "target_error_m": round(target_error, 3),
        "lateral_error_m": round(lateral_error, 3),
        "tolerance_m": round(tolerance, 3),
        **({} if completed else {
            "hint": "Motion did not finish idle within its deadline or settled "
                    "outside tolerance. Report the measured partial result; do "
                    "not claim the requested distance or retry blindly."
        }),
    }


def tool_turn_husky(husky: str = "", angle_deg: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Closed-loop relative rotation in degrees. Positive = counter-clockwise.

    The measurement is UNWRAPPED -- see _rotate_by for why that matters and
    for the two fabricated successes the wrapped version produced.
    """
    if not husky:
        return {"error": "husky is required"}
    if _ESTOP.is_set():
        return {"error": "estop_engaged", "reason": _ESTOP_REASON,
                "husky": husky,
                "hint": "Motion is disabled. Nothing moved."}
    if not _settle(husky, timeout_s=30.0):
        return {
            "error": "robot_not_idle",
            "husky": husky,
            "completed": False,
            "hint": "A previous bridge motion is still active. Nothing new was "
                    "commanded; stop or wait for that motion before retrying.",
        }
    start = _pose(husky)
    if start is None:
        return {"error": "cannot read pose", "husky": husky,
                "hint": "Nothing was commanded. Restore bridge telemetry first."}
    achieved, chunks = _rotate_by(husky, math.radians(float(angle_deg)))
    settled = _settle(husky, timeout_s=8.0)
    end = _pose(husky)
    if end is None:
        return {
            "husky": husky,
            "completed": False,
            "error": "turn accepted but final pose is unreadable",
            "hint": "Do not claim completion; telemetry verification failed.",
        }
    # UNWRAPPED, both of them. `achieved` accumulates measured per-chunk
    # rotation, and the error is a plain subtraction -- see _rotate_by.
    achieved_deg = math.degrees(achieved)
    residual_deg = achieved_deg - float(angle_deg)
    completed = settled and abs(residual_deg) <= 5.0
    return {
        "accepted": True,
        "husky": husky,
        "completed": completed,
        "settled": settled,
        "requested_angle_deg": float(angle_deg),
        "start_yaw_deg": round(math.degrees(start[2]), 2),
        "final_yaw_deg": round(math.degrees(end[2]), 2),
        "angle_achieved_deg": round(achieved_deg, 2),
        "residual_error_deg": round(residual_deg, 2),
        "rotation_chunks": chunks,
        "final_pose": {"x": round(end[0], 3), "y": round(end[1], 3),
                       "yaw": round(end[2], 4)},
        **({} if completed else {
            "hint": "Turn did not settle idle and converge within 5 degrees. "
                    "Do not claim the requested angle was reached or retry "
                    "blindly."
        }),
    }


def _norm_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _pose(husky: str) -> Optional[Tuple[float, float, float]]:
    try:
        s = _bridge_post(husky, "get_robot_state", timeout=5)
        return float(s["x"]), float(s["y"]), float(s["yaw"])
    except Exception:
        return None


# Measured open-loop gains of the bridge's motion primitives (2026-07-22,
# MuJoCo solver, supervisor ground truth): a commanded 90 deg turn lands at
# 73.2 deg (0.813) and a commanded 2.0 m drive lands at 1.83 m (0.91).
# Dividing the command by the gain lands it in ONE move instead of grinding
# the loop down 9% at a time -- iterating without compensation took 212 s to
# still miss by 0.34 m. Override if the bridge is ever recalibrated.
# Unicycle go-to-point controller gains (drive_to_xy / drive_radial).
GOTO_KV = float(os.environ.get("HUSKY_SWARM_GOTO_KV", "0.7"))
GOTO_KW = float(os.environ.get("HUSKY_SWARM_GOTO_KW", "1.8"))
GOTO_V_MAX = float(os.environ.get("HUSKY_SWARM_GOTO_V_MAX", "0.40"))
GOTO_W_MAX = float(os.environ.get("HUSKY_SWARM_GOTO_W_MAX", "1.2"))
GOTO_TIMEOUT_S = float(os.environ.get("HUSKY_SWARM_GOTO_TIMEOUT_S", "45"))
GOTO_COAST_S = float(os.environ.get("HUSKY_SWARM_GOTO_COAST_S", "0.12"))
# Below this the Husky stalls against static friction instead of creeping,
# so the approach never converges and burns the whole timeout.
GOTO_V_MIN = float(os.environ.get("HUSKY_SWARM_GOTO_V_MIN", "0.16"))
TURN_GAIN = float(os.environ.get("HUSKY_SWARM_TURN_GAIN", "0.813"))
DRIVE_GAIN = float(os.environ.get("HUSKY_SWARM_DRIVE_GAIN", "0.905"))


def _settle(husky: str, timeout_s: float = 15.0) -> bool:
    """Return only after the bridge confirms idle; never infer it from pose."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if _bridge_post(husky, "get_robot_state", timeout=5).get("mode") == "idle":
                return True
        except Exception:
            return False
        time.sleep(0.15)
    return False


# A single commanded rotation is capped well under half a turn. Everything
# downstream -- the bridge's own wrap_pi(yaw + angle) target, _turn_to_heading's
# shortest-path residual, and a pose-difference measurement -- is only
# unambiguous while |rotation| < pi, so a large request is delivered as a
# sequence of chunks rather than one command nobody can measure.
TURN_CHUNK_MAX_RAD = float(os.environ.get("HUSKY_SWARM_TURN_CHUNK_RAD", "2.0"))
TURN_CHUNK_TOL_RAD = math.radians(3.0)
TURN_MAX_CHUNKS = int(os.environ.get("HUSKY_SWARM_TURN_MAX_CHUNKS", "10"))


def _rotate_by(husky: str, angle_rad: float) -> Tuple[float, List[Dict[str, Any]]]:
    """Rotate by a SIGNED relative angle. Returns (measured_unwrapped, chunks).

    THE WRAP BLIND SPOT this exists to close: the old implementation aimed at
    `wrap_pi(start_yaw + angle)` and then measured `wrap_pi(end_yaw -
    start_yaw)`. Both wraps destroy exactly the information the caller asked
    about. Measured on a scripted husky:

      * `turn(2*pi)` -> the target equalled the start yaw, so NOTHING MOVED,
        and it reported achieved 0.00 deg, error -0.00 deg, completed true.
      * `turn(3.5 rad)` -> the shortest path is -2.78 rad, so it rotated the
        OPPOSITE WAY by -159.46 deg and reported error 0.00 deg, completed
        true, against a true error of -2*pi.

    The same defect was found and fixed in husky_omnilink_bridge (2e2471b8);
    this is the client-side twin.

    The fix, mirroring the bridge: carry a signed residual, decrement it by
    what was MEASURED, never wrap the running total. Each chunk stays under
    TURN_CHUNK_MAX_RAD so its own measurement is unambiguous, and a chunk's
    achieved rotation is `chunk - residual_to_that_chunk's_target` -- a
    subtraction of two measured quantities, with no wrap anywhere.
    """
    remaining = float(angle_rad)
    achieved_total = 0.0
    chunks: List[Dict[str, Any]] = []
    for _ in range(TURN_MAX_CHUNKS):
        if abs(remaining) <= TURN_CHUNK_TOL_RAD:
            break
        chunk = max(-TURN_CHUNK_MAX_RAD, min(TURN_CHUNK_MAX_RAD, remaining))
        here = _pose(husky)
        if here is None:
            break
        residual = _turn_to_heading(husky, _norm_angle(here[2] + chunk))
        achieved_chunk = chunk - residual
        achieved_total += achieved_chunk
        remaining -= achieved_chunk
        chunks.append({
            "commanded_deg": round(math.degrees(chunk), 2),
            "achieved_deg": round(math.degrees(achieved_chunk), 2),
            "remaining_deg": round(math.degrees(remaining), 2),
        })
        # A chunk that delivered essentially nothing means the bridge is not
        # turning; another identical command will not change that.
        if abs(achieved_chunk) < math.radians(0.5) and abs(chunk) > TURN_CHUNK_TOL_RAD:
            break
    return achieved_total, chunks


def _turn_to_heading(husky: str, target_yaw: float, tol_deg: float = 3.0,
                     max_iters: int = 3) -> float:
    """Rotate to an absolute heading, gain-compensated then verified."""
    residual = math.pi
    for _ in range(max_iters):
        p = _pose(husky)
        if p is None:
            return residual
        residual = _norm_angle(target_yaw - p[2])
        if abs(residual) <= math.radians(tol_deg):
            return residual
        command = residual / TURN_GAIN
        accepted = _bridge_post(husky, "turn", {"angle": command})
        if not isinstance(accepted, dict) or "error" in accepted:
            return residual
        # Large turns legitimately exceed the old fixed 15-second deadline.
        # Scale from the bridge's conservative angular speed, with headroom.
        settle_timeout = max(
            15.0, min(50.0, abs(command) / 0.5 * 4.5 + 5.0))
        if not _settle(husky, timeout_s=settle_timeout):
            _bridge_post(husky, "stop_robot", _bypass_estop=True)
            _settle(husky, timeout_s=8.0)
            p = _pose(husky)
            return _norm_angle(target_yaw - p[2]) if p else residual
    p = _pose(husky)
    return _norm_angle(target_yaw - p[2]) if p else residual


def tool_drive_to_xy(husky: str = "", x: float = 0.0, y: float = 0.0,
                     tolerance_m: float = 0.15, max_iters: int = 3,
                     **_: Any) -> Dict[str, Any]:
    """Drive to an absolute (x, y). Closed-loop; no trigonometry for the caller.

    This exists because the model is unreliable at per-robot heading maths --
    told to move four Huskies "outward" it sent the -x quadrants down the wrong
    diagonal. Give it a target coordinate and let this do the geometry.
    """
    if not husky:
        return {"error": "husky is required"}
    # Compound motions must report the latch themselves. Without this they run
    # their whole loop against a bridge that refuses every step and then return
    # a normal-looking {final_xy, arrived} dict -- the model reads that as
    # success and tells the operator the robot moved. Same class of defect as
    # the swallowed save_memory failure: never let the tool layer lie upward.
    if _ESTOP.is_set():
        return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": husky,
                "hint": "Motion is disabled by the operator e-stop. Nothing moved. "
                        "Report this instead of retrying."}
    x, y = float(x), float(y)
    if abs(x) > ARENA_BOUND_M or abs(y) > ARENA_BOUND_M:
        return {"error": "out_of_arena", "target_xy": [x, y],
                "arena_bound_m": ARENA_BOUND_M,
                    "estop": estop_state(),
                    "bridge_token": bool(BRIDGE_TOKEN),
                    "control_owner": _CONTROL_OWNER or None,
                "hint": f"Target is outside the +/-{ARENA_BOUND_M} m arena."}
    start = _pose(husky)
    if start is None:
        return {"error": "cannot read pose", "husky": husky}

    # Turn-then-drive, gain-compensated, using the bridge's OWN closed loops.
    # A high-rate velocity loop was tried and is worse here: each bridge runs a
    # single-threaded HTTP server sharing a lock with the 16 ms sim step, so
    # driving it at ~7 Hz per robot (x4 in parallel) starves the very pose
    # reads the controller depends on. Few, larger commands beat a fast loop.
    iters = 0
    end = start
    for _ in range(int(max_iters)):
        iters += 1
        p = _pose(husky)
        if p is None:
            break
        end = p
        dx, dy = x - p[0], y - p[1]
        remaining = math.hypot(dx, dy)
        if remaining <= tolerance_m:
            break
        _turn_to_heading(husky, math.atan2(dy, dx))
        p = _pose(husky)
        if p is None:
            break
        remaining = math.hypot(x - p[0], y - p[1])
        _bridge_post(husky, "drive_forward", {"distance": remaining / DRIVE_GAIN})
        _settle(husky)
    end = _pose(husky) or end
    err = math.hypot(x - end[0], y - end[1])
    arrived = err <= tolerance_m + 0.03
    # `completed` MIRRORS the measured verdict. It is not decoration: every
    # generic supervisor in this tree (execute_parallel's roll-up,
    # _lib.supervision.normalize_outcome, execute_verified_sequence) looks for
    # `completed`, and a tool that only says `arrived` reads to all of them as
    # "no postcondition reported" -- which is how a husky that wedged 3 m short
    # ended up inside a batch stamped all_completed: true.
    return {"husky": husky, "target_xy": [round(x, 3), round(y, 3)],
            "final_xy": [round(end[0], 3), round(end[1], 3)],
            "error_m": round(err, 3), "iterations": iters,
            "arrived": arrived,
            "completed": arrived,
            "tolerance_m": round(float(tolerance_m), 3),
            "start_xy": [round(start[0], 3), round(start[1], 3)]}


def tool_drive_radial(husky: str = "", delta_r_m: float = 0.0,
                      center_x: float = 0.0, center_y: float = 0.0,
                      **_: Any) -> Dict[str, Any]:
    """Move a Husky delta_r_m further from (positive) or nearer to (negative)
    a centre point, along its own radial direction. Deterministic."""
    if not husky:
        return {"error": "husky is required"}
    if _ESTOP.is_set():
        return {"error": "estop_engaged", "reason": _ESTOP_REASON, "husky": husky,
                "hint": "Motion is disabled by the operator e-stop. Nothing moved."}
    p = _pose(husky)
    if p is None:
        return {"error": "cannot read pose", "husky": husky}
    cx, cy = float(center_x), float(center_y)
    dx, dy = p[0] - cx, p[1] - cy
    r = math.hypot(dx, dy)
    if r < 1e-6:
        return {"error": "husky is at the centre; radial direction undefined",
                "hint": "use drive_to_xy with an explicit target instead"}
    ux, uy = dx / r, dy / r
    target_r = r + float(delta_r_m)
    if target_r < 0:
        return {"error": "delta would push past the centre", "current_r": round(r, 3)}
    res = tool_drive_to_xy(husky=husky, x=cx + ux * target_r, y=cy + uy * target_r)
    if isinstance(res, dict) and "final_xy" in res:
        fx, fy = res["final_xy"]
        res["radius_before"] = round(r, 3)
        res["radius_after"] = round(math.hypot(fx - cx, fy - cy), 3)
        res["delta_r_achieved"] = round(res["radius_after"] - r, 3)
    return res



def tool_ask_operator(question: str = "", options: str = "", **_: Any) -> Dict[str, Any]:
    """Ask the operator to resolve an ambiguity, WITHOUT moving anything.

    Models guess when guessing is the only action available. Benchmarked
    2026-07-23: told "move it forward a bit" -- no robot named, no distance --
    Flash-Lite, Grok 4.1 and Grok 4.5 each picked a robot and drove it. Giving
    the model an explicit way to ask is the fix; a prompt rule alone competes
    with the pull of the tools that DO something.
    """
    if not question:
        return {"error": "question is required"}
    return {"asked": True, "question": question,
            "options": options or None,
            "note": "Waiting for the operator. Nothing was moved. Put this "
                    "question in your reply and stop -- do not choose for them."}


def tool_find_husky(criteria: str = "", x: float = 0.0, y: float = 0.0,
                    **_: Any) -> Dict[str, Any]:
    """Answer a superlative query about the swarm from live pose.

    criteria: furthest_from_centre | nearest_to_centre | furthest_from_point |
              nearest_to_point | northmost | southmost | eastmost | westmost

    Exists because comparing four poses and computing a derived quantity is
    where models silently do nothing: on `arith_furthest` ("move the furthest
    robot to half its distance") both Grok versions moved no robot at all.
    Returns the answer AND the full ranking, so the model can verify rather
    than trust one number.
    """
    poses = {}
    for husky in BRIDGES:
        pose = _pose(husky)
        if pose is not None:
            poses[husky] = pose
    if not poses:
        return {"error": "no husky poses readable"}
    cx, cy = float(x), float(y)
    metric = {
        "furthest_from_centre": lambda p: math.hypot(p[0], p[1]),
        "nearest_to_centre": lambda p: -math.hypot(p[0], p[1]),
        "furthest_from_point": lambda p: math.hypot(p[0] - cx, p[1] - cy),
        "nearest_to_point": lambda p: -math.hypot(p[0] - cx, p[1] - cy),
        "northmost": lambda p: p[1],
        "southmost": lambda p: -p[1],
        "eastmost": lambda p: p[0],
        "westmost": lambda p: -p[0],
    }.get((criteria or "").strip().lower())
    if metric is None:
        return {"error": f"unknown criteria {criteria!r}",
                "known": ["furthest_from_centre", "nearest_to_centre",
                          "furthest_from_point", "nearest_to_point",
                          "northmost", "southmost", "eastmost", "westmost"]}
    ranked = sorted(poses.items(), key=lambda kv: metric(kv[1]), reverse=True)
    return {
        "answer": ranked[0][0],
        "criteria": criteria,
        "ranking": [
            {"husky": h, "xy": [round(pp[0], 3), round(pp[1], 3)],
             "radius_from_centre": round(math.hypot(pp[0], pp[1]), 3)}
            for h, pp in ranked
        ],
    }


def tool_turn_then_drive(husky: str = "", angle_deg: float = 0.0,
                         distance_m: float = 0.0,
                         speed_m_s: Optional[float] = None,
                         **_: Any) -> Dict[str, Any]:
    """Execute an ordered relative turn followed by a drive.

    This is deliberately one skill rather than two model-managed commands.
    The runtime owns ordering, waits for each postcondition, and stops before
    the drive if the turn could not be verified.
    """
    if not husky:
        return {"error": "husky is required"}
    start = _pose(husky)
    if start is None:
        return {"error": "cannot read pose", "husky": husky}
    turn = tool_turn_husky(husky=husky, angle_deg=angle_deg)
    if "error" in turn or not turn.get("completed"):
        return {
            "error": "turn step failed",
            "husky": husky,
            "turn": turn,
            "drive_skipped": True,
            "hint": "The ordered skill stopped safely before translation.",
        }
    drive = tool_drive_husky(
        husky=husky, distance_m=distance_m, speed_m_s=speed_m_s)
    end = _pose(husky)
    if "error" in drive or not drive.get("completed"):
        return {
            "error": "drive step failed",
            "husky": husky,
            "turn": turn,
            "drive": drive,
            "completed": False,
        }
    return {
        "husky": husky,
        "completed": True,
        "ordered_steps": ["turn", "drive"],
        "requested": {"angle_deg": float(angle_deg),
                      "distance_m": float(distance_m)},
        "turn": turn,
        "drive": drive,
        "start_pose": {"x": round(start[0], 3), "y": round(start[1], 3),
                       "yaw": round(start[2], 4)},
        "final_pose": ({"x": round(end[0], 3), "y": round(end[1], 3),
                        "yaw": round(end[2], 4)} if end else None),
    }


def tool_repeat_turn_then_drive(husky: str = "", angle_deg: float = 0.0,
                                distance_m: float = 0.0,
                                repetitions: int = 1,
                                speed_m_s: Optional[float] = None,
                                **_: Any) -> Dict[str, Any]:
    """Run dependent turn/drive legs sequentially on one robot."""
    if not husky:
        return {"error": "husky is required"}
    try:
        count = int(repetitions)
    except (TypeError, ValueError):
        return {"error": "repetitions must be an integer"}
    if count < 1 or count > 12:
        return {"error": "repetitions_out_of_range", "allowed": [1, 12]}
    start = _pose(husky)
    if start is None:
        return {"error": "cannot read pose", "husky": husky}
    sequence = execute_verified_sequence(
        [{"id": f"leg_{index + 1}"} for index in range(count)],
        lambda _step: tool_turn_then_drive(
            husky=husky,
            angle_deg=angle_deg,
            distance_m=distance_m,
            speed_m_s=speed_m_s,
        ),
        verifier=lambda _step, result: (
            isinstance(result, dict)
            and "error" not in result
            and result.get("completed") is True
        ),
    )
    legs = [entry["result"] for entry in sequence["steps"]]
    if not sequence["completed"]:
        return {
            "error": "repeated_motion_failed",
            "husky": husky,
            "completed": False,
            "verified": False,
            "completed_legs": sequence["completed_steps"],
            "requested_repetitions": count,
            "failed_leg": len(legs),
            "ledger": sequence,
            "legs": legs,
            "hint": "The sequence stopped at the first unverified leg.",
        }
    end = _pose(husky)
    return {
        "husky": husky,
        "completed": True,
        "verified": True,
        "completed_legs": count,
        "requested_repetitions": count,
        "requested_per_leg": {
            "angle_deg": float(angle_deg), "distance_m": float(distance_m)},
        "legs": legs,
        "ledger": sequence,
        "start_pose": {"x": round(start[0], 3), "y": round(start[1], 3),
                       "yaw": round(start[2], 4)},
        "final_pose": ({"x": round(end[0], 3), "y": round(end[1], 3),
                        "yaw": round(end[2], 4)} if end else None),
    }


def tool_find_and_drive(criteria: str = "", distance_m: float = 0.0,
                        x: float = 0.0, y: float = 0.0,
                        speed_m_s: Optional[float] = None,
                        **_: Any) -> Dict[str, Any]:
    """Select one robot from live telemetry and drive exactly that robot.

    Selection and actuation share one runtime transaction, so the model cannot
    accidentally copy the wrong name from a ranking or act on stale spawn
    coordinates.  Useful for "move the nearest/furthest/northmost..." tasks.
    """
    selected = tool_find_husky(criteria=criteria, x=x, y=y)
    if "error" in selected:
        return selected
    husky = str(selected["answer"])
    drive = tool_drive_husky(
        husky=husky, distance_m=distance_m, speed_m_s=speed_m_s)
    if "error" in drive:
        return {"error": "selected robot did not move",
                "selected_husky": husky, "selection": selected,
                "drive": drive}
    return {
        "completed": bool(drive.get("completed")),
        "selected_husky": husky,
        "criteria": criteria,
        "selection": selected,
        "drive": drive,
    }


def tool_move_swarm_to(targets: str = "", **_: Any) -> Dict[str, Any]:
    """Drive the swarm to a set of target points, assigning robots optimally.

    targets: "x1,y1; x2,y2; ..." -- one point per robot to be moved.

    Each robot is matched to its nearest unclaimed target (greedy on the
    shortest remaining pair), then all moves run in parallel. Without this the
    model must both invent the coordinates AND decide who goes where; on
    `form_line` that produced two robots stacked on one point and two left in
    place. Assignment is a solved problem -- it should not be the model's job.
    """
    pts = []
    for chunk in re.split(r"[;\n]", targets or ""):
        chunk = chunk.strip().strip("()[]")
        if not chunk:
            continue
        try:
            a, b = chunk.split(",")[:2]
            pts.append((float(a), float(b)))
        except (ValueError, IndexError):
            return {"error": f"could not parse target {chunk!r}",
                    "format": "x1,y1; x2,y2; ..."}
    if not pts:
        return {"error": "targets is required", "format": "x1,y1; x2,y2; ..."}
    poses = {h: _pose(h) for h in BRIDGES}
    poses = {h: p for h, p in poses.items() if p is not None}
    if not poses:
        return {"error": "no husky poses readable"}

    pairs = sorted(
        ((math.dist((p[0], p[1]), pt), h, i)
         for h, p in poses.items() for i, pt in enumerate(pts)),
        key=lambda z: z[0])
    assigned: Dict[str, int] = {}
    used: set = set()
    for _d, h, i in pairs:
        if h in assigned or i in used:
            continue
        assigned[h] = i
        used.add(i)
    actions = [{"tool": "drive_to_xy",
                "args": {"husky": h, "x": pts[i][0], "y": pts[i][1]}}
               for h, i in assigned.items()]
    result = tool_execute_parallel(actions=actions)
    unassigned = [list(pts[i]) for i in range(len(pts)) if i not in used]
    return {"assignment": {h: list(pts[i]) for h, i in assigned.items()},
            "unassigned_targets": unassigned,
            # Forward the batch verdict instead of swallowing it: returning
            # only `results` left the caller to re-derive success from four
            # nested dicts, which is exactly the re-derivation the model gets
            # wrong.
            "completed": bool(result.get("all_completed")) and not unassigned,
            "failure_indexes": result.get("failure_indexes"),
            "unverified_indexes": result.get("unverified_indexes"),
            "results": result.get("results")}



def tool_count_huskies(region: str = "", radius_m: float = 0.0,
                       x: float = 0.0, y: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Count the robots matching a spatial predicate, from live pose.

    region: west (x<0) | east (x>0) | north (y>0) | south (y<0) |
            within_radius | beyond_radius | all

    Exists because "how many are in the western half, then move that many"
    requires a count the model must otherwise derive by eyeballing four
    coordinate pairs. Benchmarked 2026-07-23: on that task one model moved a
    single robot and the other moved none. Returns the count AND the names, so
    the follow-up action can name them explicitly instead of re-deriving.
    """
    poses = {h: _pose(h) for h in BRIDGES}
    poses = {h: p for h, p in poses.items() if p is not None}
    if not poses:
        return {"error": "no husky poses readable"}
    r = float(radius_m)
    cx, cy = float(x), float(y)
    tests = {
        "west": lambda p: p[0] < 0, "east": lambda p: p[0] > 0,
        "north": lambda p: p[1] > 0, "south": lambda p: p[1] < 0,
        "within_radius": lambda p: math.hypot(p[0] - cx, p[1] - cy) <= r,
        "beyond_radius": lambda p: math.hypot(p[0] - cx, p[1] - cy) > r,
        "all": lambda p: True,
    }
    test = tests.get((region or "").strip().lower())
    if test is None:
        return {"error": f"unknown region {region!r}", "known": sorted(tests)}
    matched = sorted(h for h, p in poses.items() if test(p))
    return {"region": region, "count": len(matched), "huskies": matched,
            "all_positions": {h: [round(p[0], 3), round(p[1], 3)]
                              for h, p in sorted(poses.items())}}


def tool_set_husky_velocity(husky: str = "", linear: float = 0.0,
                            angular: float = 0.0, **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
    # Continuous velocity was the one unguarded motion path: the bridge lets a
    # twist run for VELOCITY_MAX_S (12 s) before it expires, which at cruise is
    # ~6 m of travel with no arena check -- straight through the wall while
    # drive_husky next to it refuses the same distance. Guard it on the
    # WORST CASE (full expiry at the commanded speed) rather than on the
    # instantaneous command, because nothing guarantees a follow-up stop.
    projected = abs(float(linear)) * VELOCITY_EXPIRY_S
    if projected > 0:
        blocked = _guard_drive_distance(husky, math.copysign(projected, float(linear)))
        if blocked is not None:
            blocked["note"] = (
                f"set_velocity runs for up to {VELOCITY_EXPIRY_S:.0f}s before it "
                f"expires, so linear={linear} could travel {projected:.1f} m. Use "
                f"drive_husky or drive_to_xy for a bounded move.")
            return blocked
    return _bridge_post(husky, "set_velocity",
                        {"linear": float(linear), "angular": float(angular)})


def tool_stop_husky(husky: str = "", **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
    with _METRICS_LOCK:
        _METRICS[husky]["halts"] = _METRICS[husky].get("halts", 0) + 1
    return _bridge_post(husky, "stop_robot")


def tool_reset_husky_to_home(husky: str = "", **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
    return _bridge_post(husky, "reset_to_home")


def tool_wait_for_husky_idle(husky: str = "", timeout_s: float = 10.0,
                             poll_hz: float = 5.0, **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
    period = 1.0 / max(poll_hz, 1e-3)
    t0 = time.time()
    state: Dict[str, Any] = {}
    while time.time() - t0 < timeout_s:
        state = _bridge_post(husky, "get_robot_state", timeout=2)
        # Accumulate per-Husky distance during the poll loop so metrics
        # reflect actual motion even when the operator never asks for
        # get_swarm_state directly.
        if isinstance(state, dict) and "x" in state and "y" in state:
            _update_metrics_from_state({husky: state})
        if state.get("mode") == "idle":
            return {"idle": True, "waited_s": round(time.time() - t0, 2),
                    "final_state": state}
        time.sleep(period)
    return {"idle": False, "waited_s": round(time.time() - t0, 2),
            "final_state": state, "note": f"timeout after {timeout_s}s"}


def tool_halt_all(**_: Any) -> Dict[str, Any]:
    """Emergency stop in parallel across every Husky."""
    results: Dict[str, Any] = {}
    threads = []
    def _halt(name: str) -> None:
        results[name] = _bridge_post(name, "stop_robot", timeout=2)
    for h in sorted(BRIDGES.keys()):
        t = threading.Thread(target=_halt, args=(h,), daemon=True); t.start()
        threads.append(t)
    for t in threads: t.join(timeout=5)
    with _METRICS_LOCK:
        for h in BRIDGES:
            _METRICS[h]["halts"] = _METRICS[h].get("halts", 0) + 1
    return {"halted": results, "timestamp": time.time()}


# ─── execute_parallel: the multi-robot superpower ──────────────────

# A postcondition key a tool sets itself, having MEASURED it. `accepted` is
# deliberately absent -- an acknowledgement is not an arrival.
_PARALLEL_VERDICT_KEYS = ("completed", "arrived", "within_tolerance")


def _parallel_verdict(spec: Any, result: Any) -> str:
    """Classify one parallel slot: 'failed' | 'verified' | 'unverified'.

    The old rule was negative -- a slot counted as a failure only if it said
    `error` or `completed is False`. The two motion tools the shipped prompt
    puts inside execute_parallel (drive_to_xy, drive_radial) said neither:
    they reported `arrived`, so "fan out to the corners" with one husky wedged
    3 m short returned all_completed: true, failure_indexes: [], and
    normalize_outcome then stamped the batch verified. The fleet was not
    contained and the model told the operator four robots arrived.

    So the rule is now POSITIVE: a slot is verified only when the tool itself
    published a measured postcondition that is True. A motion tool that
    published none is 'unverified' -- reported, but never counted as success.
    Non-actuating tools (telemetry, memory, meta) have no postcondition to
    publish, so a clean return is all they can offer and all we require.
    """
    if not isinstance(result, dict):
        return "failed"
    if failure_error(result):
        return "failed"
    for key in _PARALLEL_VERDICT_KEYS:
        value = result.get(key)
        if isinstance(value, bool):
            return "verified" if value else "failed"
    tool_name = spec.get("tool", "") if isinstance(spec, dict) else ""
    if "motion" in ((TOOLS.get(tool_name) or {}).get("tags") or []):
        return "unverified"
    return "verified"


def tool_execute_parallel(actions: Optional[List[Dict[str, Any]]] = None,
                          halt_on_error: bool = False,
                          **_: Any) -> Dict[str, Any]:
    """Dispatch N tool calls in parallel. Returns per-action results
    keyed by index. The agent's primary way to issue swarm-wide
    commands in one network round-trip instead of N sequential ones."""
    if not actions or not isinstance(actions, list):
        return {"error": "actions must be a non-empty list of {tool, args}"}
    duplicates = find_duplicate_resource_claims(
        actions,
        lambda spec: (
            str((spec.get("args") or {}).get("husky") or "")
            if isinstance(spec, dict) and isinstance(spec.get("args") or {}, dict)
            else None
        ),
    )
    if duplicates:
        duplicate = duplicates[0]
        return {
            "error": "duplicate_robot_action",
            "husky": duplicate["resource"],
            "first_index": duplicate["first_index"],
            "duplicate_index": duplicate["duplicate_index"],
            "duplicate_claims": duplicates,
            "hint": "Parallel calls must target distinct robots. Use "
                    "repeat_turn_then_drive for dependent repeated legs on "
                    "one robot.",
        }
    results: List[Optional[Dict[str, Any]]] = [None] * len(actions)
    threads: List[threading.Thread] = []
    def _run(i: int, spec: Dict[str, Any]) -> None:
        try:
            tname = spec.get("tool", "")
            targs = spec.get("args", {}) or {}
            # Only dispatch tools that don't recursively re-enter
            # execute_parallel (avoids stack blowups and circular loops).
            if tname == "execute_parallel":
                results[i] = {"error": "execute_parallel cannot be nested"}
                return
            results[i] = dispatch(tname, dict(targs), gate=False)
        except Exception as e:
            results[i] = {"error": f"parallel slot {i} crashed: {e}"}
    for i, spec in enumerate(actions):
        t = threading.Thread(target=_run, args=(i, spec), daemon=True); t.start()
        threads.append(t)
    # Compound motions (drive_to_xy / drive_radial) legitimately run far longer
    # than one bridge call. Joining at BRIDGE_TIMEOUT_S*2 returned four bare
    # nulls while the threads kept driving in the background -- the results
    # were lost AND the next command raced still-moving robots.
    deadline = time.time() + PARALLEL_JOIN_TIMEOUT_S
    for t in threads:
        t.join(timeout=max(1.0, deadline - time.time()))
    for i, t in enumerate(threads):
        if t.is_alive() and results[i] is None:
            results[i] = {"error": "timed_out",
                          "detail": f"action {i} still running after "
                                    f"{PARALLEL_JOIN_TIMEOUT_S:.0f}s",
                          "hint": "The robot may still be moving. Call "
                                   "get_swarm_state before assuming anything."}
    verdicts = [_parallel_verdict(actions[i], result)
                for i, result in enumerate(results)]
    failures = [i for i, v in enumerate(verdicts) if v == "failed"]
    unverified = [i for i, v in enumerate(verdicts) if v == "unverified"]
    halted = None
    if failures and halt_on_error:
        halted = tool_halt_all()
    return {
        "results": results,
        "count": len(actions),
        "failure_indexes": failures,
        "unverified_indexes": unverified,
        "verified_indexes": [i for i, v in enumerate(verdicts) if v == "verified"],
        # POSITIVE verification only: every slot measured its own
        # postcondition and it held. An unverified slot is not a success.
        "all_completed": not failures and not unverified,
        "halted_on_error": bool(halted),
        **({"halt_result": halted} if halted else {}),
        **({"hint": (
            f"{len(unverified)} action(s) returned no measured postcondition "
            f"(indexes {unverified}). They are NOT proof of completion -- read "
            f"get_swarm_state before telling the operator anything about them."
        )} if unverified else {}),
        "timestamp": time.time(),
    }


# ─── Waypoints (persistent named spatial coordinates) ──────────────

def tool_save_waypoint(name: str = "", x: Optional[float] = None,
                       y: Optional[float] = None, husky: str = "",
                       note: str = "", **_: Any) -> Dict[str, Any]:
    """Save a named (x, y) waypoint. Either supply x+y directly OR pass
    husky=<id> to snapshot the CURRENT pose of that Husky. Persistent
    across sessions."""
    if not name:
        return {"error": "name is required"}
    if x is None or y is None:
        if not husky:
            return {"error": "must provide (x, y) or husky=<id> to snapshot"}
        st = _bridge_post(husky, "get_robot_state", timeout=2)
        if "x" not in st or "y" not in st:
            return {"error": f"could not read {husky}'s pose", "detail": st}
        x = float(st["x"]); y = float(st["y"])
    _store_write(
        "INSERT INTO waypoints(name, x, y, note, updated_at) VALUES (?, ?, ?, ?, ?) "
        "ON CONFLICT(name) DO UPDATE SET x=excluded.x, y=excluded.y, "
        "note=excluded.note, updated_at=excluded.updated_at",
        (name, float(x), float(y), note or "", _now_iso())
    )
    return {"saved": True, "name": name, "x": float(x), "y": float(y), "note": note}


def tool_list_waypoints(**_: Any) -> Dict[str, Any]:
    rows = _store_exec("SELECT name, x, y, note, updated_at FROM waypoints ORDER BY name")
    return {"waypoints": rows, "count": len(rows)}


def tool_recall_waypoint(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name:
        return {"error": "name is required"}
    rows = _store_exec("SELECT name, x, y, note, updated_at FROM waypoints WHERE name=?", (name,))
    if not rows:
        return {"error": f"no waypoint named {name!r}",
                "hint": "call list_waypoints to see what's saved"}
    return rows[0]


def tool_forget_waypoint(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name:
        return {"error": "name is required"}
    _store_write("DELETE FROM waypoints WHERE name=?", (name,))
    return {"forgotten": True, "name": name}


def tool_drive_to_waypoint(husky: str = "", name: str = "",
                           speed_m_s: Optional[float] = None,
                           tolerance_m: float = 0.4,
                           max_iters: int = 6,
                           segment_m: float = 1.2, **_: Any) -> Dict[str, Any]:
    """Drive one Husky to a named waypoint via iterative point-and-shoot.

    Husky is a skid-steer; turn-in-place drifts ~1-3 m forward during
    big rotations, and drive_forward overshoots by ~10-20% due to wheel
    momentum. Strategy: short segments (cap at `segment_m`). Each
    iteration: re-read pose, compute remaining bearing + distance, turn
    if needed (post-turn drift is accepted), drive at most segment_m
    forward, repeat. Convergence guaranteed because each segment's
    overshoot is bounded by segment_m, and the next iteration's bearing
    correction handles direction error. Stops when within tolerance
    or max_iters exhausted."""
    if not husky:
        return {"error": "husky is required"}
    if not name:
        return {"error": "name is required"}
    wp_rows = _store_exec("SELECT x, y FROM waypoints WHERE name=?", (name,))
    if not wp_rows:
        return {"error": f"no waypoint named {name!r}"}
    tx = float(wp_rows[0]["x"]); ty = float(wp_rows[0]["y"])
    if abs(tx) > ARENA_BOUND_M or abs(ty) > ARENA_BOUND_M:
        return {"error": "waypoint out of arena", "target": [tx, ty]}

    iters: List[Dict[str, Any]] = []
    final_err = float("inf")
    final_xy: List[float] = [0.0, 0.0]
    final_within = False

    for it in range(max_iters):
        st = _bridge_post(husky, "get_robot_state", timeout=2)
        if "x" not in st:
            return {"error": "could not read state", "detail": st}
        cx, cy = float(st["x"]), float(st["y"])
        yaw = float(st.get("yaw", 0.0))
        dx, dy = tx - cx, ty - cy
        distance = math.hypot(dx, dy)
        final_xy = [round(cx, 3), round(cy, 3)]
        final_err = distance
        if distance < tolerance_m:
            iters.append({"iter": it, "from": [round(cx, 2), round(cy, 2)],
                          "remaining_m": round(distance, 3), "action": "within_tolerance"})
            final_within = True
            break

        target_yaw = math.atan2(dy, dx)
        turn_rad = target_yaw - yaw
        while turn_rad > math.pi: turn_rad -= 2 * math.pi
        while turn_rad < -math.pi: turn_rad += 2 * math.pi

        leg: Dict[str, Any] = {
            "iter": it, "from": [round(cx, 2), round(cy, 2)],
            "yaw": round(yaw, 2), "remaining_m": round(distance, 3),
            "turn_rad": round(turn_rad, 2),
        }

        # Turn only if the correction is non-trivial -- avoids the
        # in-place-drift problem on small final-approach corrections.
        if abs(turn_rad) > 0.08:
            _bridge_post(husky, "turn", {"angle": turn_rad})
            turn_wait_s = max(abs(turn_rad) / 0.5 * 4.5 + 5.0, 15.0)
            tres = tool_wait_for_husky_idle(husky=husky, timeout_s=turn_wait_s)
            if not tres.get("idle"):
                leg["turn_failed"] = True
                iters.append(leg)
                return {"error": "turn_did_not_complete", "iter": it,
                        "iterations": iters, "wait_result": tres}

        # Re-read pose AFTER the turn (the husky drifts during it on a
        # skid-steer) so the drive leg uses the post-turn distance,
        # not the pre-turn one. THIS is the fix for the overshoot bug.
        st2 = _bridge_post(husky, "get_robot_state", timeout=2)
        cx2, cy2 = float(st2.get("x", cx)), float(st2.get("y", cy))
        drive_dist = math.hypot(tx - cx2, ty - cy2)
        leg["after_turn_xy"] = [round(cx2, 2), round(cy2, 2)]
        leg["drive_dist"] = round(drive_dist, 3)

        if drive_dist < tolerance_m:
            final_xy = [round(cx2, 3), round(cy2, 3)]
            final_err = drive_dist
            final_within = True
            iters.append(leg)
            break

        # Cap each drive leg at segment_m so overshoot stays bounded.
        # Remaining iterations cover the rest.
        seg = min(drive_dist, segment_m)
        leg["seg_m"] = round(seg, 3)
        payload: Dict[str, Any] = {"distance": seg}
        if speed_m_s is not None:
            payload["speed"] = float(speed_m_s)
        _bridge_post(husky, "drive_forward", payload)
        drive_wait_s = max(seg / 0.3 * 3 + 5, 15.0)
        tool_wait_for_husky_idle(husky=husky, timeout_s=drive_wait_s)
        iters.append(leg)

    return {
        "name": name, "target_xy": [tx, ty], "final_xy": final_xy,
        "error_m": round(final_err, 3),
        "within_tolerance": bool(final_within or final_err < tolerance_m),
        "iterations": iters, "iter_count": len(iters),
        "note": (
            "skid-steer Husky has ~1 m residual error on open-loop nav. "
            "Chain another drive_to_waypoint call if tighter precision needed."
            if final_err > tolerance_m else ""
        ),
    }


def tool_visit_and_return(husky: str = "", x: float = 0.0, y: float = 0.0,
                          checkpoint_name: str = "return_checkpoint",
                          tolerance_m: float = 0.35,
                          **_: Any) -> Dict[str, Any]:
    """Atomically snapshot live state, visit a point, then restore it.

    The checkpoint is written before any motion.  The outbound and return legs
    are verified independently, and a failed outbound leg never starts a blind
    return. This is the general transaction for inspections, deliveries and
    temporary diversions that must leave the robot where it began.
    """
    if not husky:
        return {"error": "husky is required"}
    if not checkpoint_name:
        return {"error": "checkpoint_name is required"}
    saved = tool_save_waypoint(
        name=checkpoint_name, husky=husky,
        note="automatic live checkpoint for visit_and_return")
    if "error" in saved:
        return {"error": "checkpoint_failed", "checkpoint": saved,
                "checkpoint_saved": False}
    outbound = tool_drive_to_xy(
        husky=husky, x=float(x), y=float(y), tolerance_m=tolerance_m)
    outbound_ok = bool(outbound.get("arrived")) and "error" not in outbound
    if not outbound_ok:
        return {
            "error": "outbound_failed",
            "checkpoint_saved": True,
            "checkpoint": saved,
            "outbound_completed": False,
            "outbound": outbound,
            "return_attempted": False,
        }
    returned = tool_drive_to_waypoint(
        husky=husky, name=checkpoint_name, tolerance_m=tolerance_m)
    return_ok = bool(returned.get("within_tolerance")) and "error" not in returned
    return {
        **({} if return_ok else {"error": "return_failed"}),
        "husky": husky,
        "checkpoint_name": checkpoint_name,
        "checkpoint_saved": True,
        "checkpoint": saved,
        "outbound_completed": True,
        "outbound": outbound,
        "return_attempted": True,
        "return_completed": return_ok,
        "return": returned,
        "completed": return_ok,
    }


def tool_drive_with_fallback(husky: str = "", distance_m: float = 0.0,
                             fallback_x: float = 0.0,
                             fallback_y: float = 0.0,
                             **_: Any) -> Dict[str, Any]:
    """Try one bounded drive and use an explicit fallback only on refusal."""
    primary = tool_drive_husky(husky=husky, distance_m=distance_m)
    if "error" not in primary:
        return {
            "husky": husky,
            "primary_refused": False,
            "fallback_attempted": False,
            "primary": primary,
            "completed": bool(primary.get("completed")),
        }
    blob = json.dumps(primary, default=str).lower()
    guard_refusal = any(marker in blob for marker in (
        "out_of_arena", "arena_bound_m", "outside", "guard"))
    if not guard_refusal:
        return {
            "error": "primary_failed_without_guard_refusal",
            "husky": husky,
            "primary_refused": False,
            "fallback_attempted": False,
            "primary": primary,
        }
    fallback = tool_drive_to_xy(
        husky=husky, x=float(fallback_x), y=float(fallback_y))
    fallback_ok = bool(fallback.get("arrived")) and "error" not in fallback
    return {
        **({} if fallback_ok else {"error": "fallback_failed"}),
        "husky": husky,
        "primary_refused": True,
        "primary": primary,
        "fallback_attempted": True,
        "fallback": fallback,
        "completed": fallback_ok,
    }


# ─── Routines (persistent multi-step plans) ────────────────────────

def tool_save_routine(name: str = "", steps: Optional[List[Dict[str, Any]]] = None,
                      note: str = "", **_: Any) -> Dict[str, Any]:
    """Save a named multi-step routine. Each step is a {tool, args} dict.
    Execute later with run_routine(name). Persistent across sessions."""
    if not name:
        return {"error": "name is required"}
    if not steps or not isinstance(steps, list):
        return {"error": "steps must be a non-empty list of {tool, args}"}
    _store_write(
        "INSERT INTO routines(name, steps_json, note, updated_at) VALUES (?, ?, ?, ?) "
        "ON CONFLICT(name) DO UPDATE SET steps_json=excluded.steps_json, "
        "note=excluded.note, updated_at=excluded.updated_at",
        (name, json.dumps(steps), note or "", _now_iso())
    )
    return {"saved": True, "name": name, "step_count": len(steps), "note": note}


def tool_list_routines(**_: Any) -> Dict[str, Any]:
    rows = _store_exec("SELECT name, note, updated_at FROM routines ORDER BY name")
    # Also count steps without ballooning the response with every step body.
    full = _store_exec("SELECT name, steps_json FROM routines")
    counts = {r["name"]: len(json.loads(r["steps_json"])) for r in full}
    for r in rows:
        r["step_count"] = counts.get(r["name"], 0)
    return {"routines": rows, "count": len(rows)}


def tool_describe_routine(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name:
        return {"error": "name is required"}
    rows = _store_exec("SELECT name, steps_json, note, updated_at FROM routines WHERE name=?", (name,))
    if not rows:
        return {"error": f"no routine named {name!r}"}
    r = rows[0]
    r["steps"] = json.loads(r.pop("steps_json"))
    return r


def tool_run_routine(name: str = "", stop_on_error: bool = True, **_: Any) -> Dict[str, Any]:
    """Execute a saved routine step by step. Each step is dispatched
    just like a direct tool call. Aborts on the first error if
    stop_on_error is True. Logs elapsed time so the agent can compare
    runs across sessions."""
    if not name:
        return {"error": "name is required"}
    rows = _store_exec("SELECT steps_json FROM routines WHERE name=?", (name,))
    if not rows:
        return {"error": f"no routine named {name!r}"}
    steps = json.loads(rows[0]["steps_json"])
    t0 = time.time()
    results: List[Any] = []
    failure: Optional[Dict[str, Any]] = None
    for i, step in enumerate(steps):
        tname = step.get("tool", "")
        targs = step.get("args", {}) or {}
        try:
            res = dispatch(tname, dict(targs), gate=False)
        except Exception as e:
            res = {"error": f"step {i} crashed: {e}"}
        results.append({"step": i, "tool": tname, "result_summary": _summarize_result(res)})
        if "error" in res and stop_on_error:
            failure = {"failed_at_step": i, "tool": tname, "error": res["error"]}
            break
    elapsed = round(time.time() - t0, 2)
    return {
        "name": name, "step_count": len(steps), "executed_count": len(results),
        "elapsed_s": elapsed, "results": results, "failure": failure,
    }


def tool_forget_routine(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name:
        return {"error": "name is required"}
    _store_write("DELETE FROM routines WHERE name=?", (name,))
    return {"forgotten": True, "name": name}


# ─── Free-form memory (key/value swarm facts) ──────────────────────

def tool_save_memory(key: str = "", value: str = "", tags: str = "", **_: Any) -> Dict[str, Any]:
    if not key:
        return {"error": "key is required"}
    if not value:
        return {"error": "value is required"}
    _store_write(
        "INSERT INTO memories(key, value, tags, updated_at) VALUES (?, ?, ?, ?) "
        "ON CONFLICT(key) DO UPDATE SET value=excluded.value, tags=excluded.tags, "
        "updated_at=excluded.updated_at",
        (key, value, tags or "", _now_iso())
    )
    return {"saved": True, "key": key, "tags": tags}


def tool_recall_memory(key: str = "", **_: Any) -> Dict[str, Any]:
    if not key:
        return {"error": "key is required"}
    rows = _store_exec("SELECT key, value, tags, updated_at FROM memories WHERE key=?", (key,))
    if not rows:
        return {"error": f"no memory for key {key!r}"}
    return rows[0]


def tool_list_memories(prefix: str = "", tag: str = "", **_: Any) -> Dict[str, Any]:
    if tag:
        rows = _store_exec(
            "SELECT key, value, tags, updated_at FROM memories WHERE tags LIKE ? ORDER BY key",
            (f"%{tag}%",)
        )
    elif prefix:
        rows = _store_exec(
            "SELECT key, value, tags, updated_at FROM memories WHERE key LIKE ? ORDER BY key",
            (f"{prefix}%",)
        )
    else:
        rows = _store_exec("SELECT key, value, tags, updated_at FROM memories ORDER BY key")
    return {"memories": rows, "count": len(rows)}


def tool_forget_memory(key: str = "", **_: Any) -> Dict[str, Any]:
    if not key:
        return {"error": "key is required"}
    _store_write("DELETE FROM memories WHERE key=?", (key,))
    return {"forgotten": True, "key": key}


# ─── Outcomes (per-task success history; the self-improvement seed) ─

def tool_report_outcome(task: str = "", success: bool = False, notes: str = "",
                        elapsed_s: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Record the outcome of a task. Use after running a routine or
    multi-step plan so future sessions can learn from past attempts."""
    if not task:
        return {"error": "task is required"}
    _store_write(
        "INSERT INTO outcomes(task, success, notes, elapsed_s, recorded_at) "
        "VALUES (?, ?, ?, ?, ?)",
        (task, 1 if success else 0, notes or "", float(elapsed_s), _now_iso())
    )
    return {"recorded": True, "task": task, "success": bool(success)}


def tool_recall_past_attempts(task_keyword: str = "", limit: int = 5, **_: Any) -> Dict[str, Any]:
    if not task_keyword:
        rows = _store_exec(
            "SELECT id, task, success, notes, elapsed_s, recorded_at FROM outcomes "
            "ORDER BY id DESC LIMIT ?", (int(limit),)
        )
    else:
        rows = _store_exec(
            "SELECT id, task, success, notes, elapsed_s, recorded_at FROM outcomes "
            "WHERE task LIKE ? ORDER BY id DESC LIMIT ?",
            (f"%{task_keyword}%", int(limit))
        )
    return {"attempts": rows, "count": len(rows)}


# ─── Observability ──────────────────────────────────────────────────

def tool_get_activity_log(last_n: int = 20, tool: str = "", **_: Any) -> Dict[str, Any]:
    n = max(1, min(int(last_n), ACTIVITY_LOG_MAX))
    snap = list(ACTIVITY_LOG)
    if tool:
        snap = [e for e in snap if e.get("tool") == tool]
    return {"entries": snap[-n:], "count": min(n, len(snap))}


def tool_get_swarm_metrics(**_: Any) -> Dict[str, Any]:
    with _METRICS_LOCK:
        snap = {h: dict(m) for h, m in _METRICS.items()}
    return {"metrics": snap, "timestamp": time.time()}


def tool_reset_swarm_metrics(**_: Any) -> Dict[str, Any]:
    with _METRICS_LOCK:
        for h in BRIDGES:
            _METRICS[h] = {"distance_m": 0.0, "halts": 0, "tool_calls": 0}
        _METRICS_LAST_POSE.clear()
    return {"reset": True}


# ─── Meta tools (meta-tool pattern) ──────────────────────────────────

def tool_list_tools(tag: str = "", **_: Any) -> Dict[str, Any]:
    """Enumerate every tool the agent has, optionally filtered by tag."""
    out = []
    for n, spec in TOOLS.items():
        tags = spec.get("tags", [])
        if tag and tag not in tags:
            continue
        out.append({"name": n,
                    "description": spec["description"].split(".")[0] + ".",
                    "tags": tags})
    return {"tools": out, "count": len(out)}


def tool_find_tools(query: str = "", limit: int = 8, **_: Any) -> Dict[str, Any]:
    """Search tools by keyword in name / description / tags. Use when
    you're not sure which primitive to reach for."""
    if not query:
        return {"error": "query is required"}
    q = query.lower()
    scored = []
    for n, spec in TOOLS.items():
        score = 0
        if q in n.lower(): score += 5
        if q in spec["description"].lower(): score += 2
        if any(q in t.lower() for t in spec.get("tags", [])): score += 3
        if score > 0:
            scored.append((score, n, spec))
    scored.sort(key=lambda r: -r[0])
    out = [{"name": n, "description": spec["description"].split(".")[0] + ".",
            "tags": spec.get("tags", []), "score": s}
           for s, n, spec in scored[:int(limit)]]
    return {"matches": out, "count": len(out)}


def tool_invoke_tool(name: str = "", args: Optional[Dict[str, Any]] = None, **_: Any) -> Dict[str, Any]:
    """Dispatch any registered tool by name. Useful when the agent
    composed a plan whose tools it doesn't have in the manifest yet."""
    if not name:
        return {"error": "name is required"}
    return dispatch(name, args or {}, gate=False)


# ─── Tool registry ──────────────────────────────────────────────────

TOOLS: Dict[str, Dict[str, Any]] = {
    # ── Bridge primitives ──
    "list_huskies": {
        "tags": ["bridge", "discovery", "essential"],
        "description": "Probe every Husky bridge. Returns per-Husky {online, id, model}. Call once at the start of a session.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_huskies,
    },
    "get_swarm_state": {
        "tags": ["bridge", "telemetry", "essential"],
        "description": "One-call snapshot of all 4 Huskies: x, y, yaw, v_linear, v_angular, mode, fault. Updates internal distance metrics.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_get_swarm_state,
    },
    "get_husky_status": {
        "tags": ["bridge", "telemetry", "essential"],
        "description": (
            "READ-ONLY live state for one Husky: x, y, yaw, velocity and mode. "
            "Use this for every 'where is <husky> now?' request; never answer "
            "from spawn coordinates or memory."
        ),
        "parameters": {"type": "object", "properties": {"husky": {"type": "string"}}, "required": ["husky"]},
        "impl": tool_get_husky_status,
    },
    "delegate_to_husky": {
        "tags": ["bridge", "passthrough"],
        "description": "Natural-language task forwarded to one Husky's /prompt endpoint. The per-Husky agent handles it.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"}, "task": {"type": "string"},
        }, "required": ["husky", "task"]},
        "impl": tool_delegate_to_husky,
    },
    "drive_husky": {
        "tags": ["motion", "essential"],
        "description": (
            "Verified straight-line motion in metres along the current heading. "
            "Arena-bound-guarded; positive=forward. Blocks until idle and returns "
            "measured start_pose, final_pose and distance_achieved_m, so no "
            "separate wait tool is needed."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "distance_m": {"type": "number"},
            "speed_m_s": {"type": "number"},
        }, "required": ["husky", "distance_m"]},
        "impl": tool_drive_husky,
    },
    "turn_husky": {
        "tags": ["motion", "essential"],
        "description": (
            "Closed-loop relative rotation in degrees. Positive=counter-clockwise. "
            "Corrects actuator undershoot, blocks until idle, and returns the "
            "measured angle, residual error and final pose. "
            "angle_achieved_deg and residual_error_deg are UNWRAPPED, so a "
            "request beyond +/-180 deg (or a full 360) is expressible and "
            "verifiable; large rotations are delivered as several bounded "
            "sub-turns, listed in rotation_chunks."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "angle_deg": {"type": "number"},
        }, "required": ["husky", "angle_deg"]},
        "impl": tool_turn_husky,
    },
    "turn_then_drive": {
        "tags": ["motion", "composite", "essential"],
        "description": (
            "Ordered two-step skill: turn one Husky, verify the turn completed, "
            "then drive along the new heading and verify the final pose. USE "
            "THIS whenever an instruction says turn/rotate THEN drive/move."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "angle_deg": {"type": "number"},
            "distance_m": {"type": "number"},
            "speed_m_s": {"type": "number"},
        }, "required": ["husky", "angle_deg", "distance_m"]},
        "impl": tool_turn_then_drive,
    },
    "repeat_turn_then_drive": {
        "tags": ["motion", "composite", "essential"],
        "description": (
            "Run the same ordered turn-then-drive leg repeatedly and "
            "SEQUENTIALLY on one Husky, verifying every leg. USE THIS for "
            "squares, polygons, or 'repeat N times'. Never put multiple actions "
            "for one robot inside execute_parallel."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "angle_deg": {"type": "number"},
            "distance_m": {"type": "number"},
            "repetitions": {"type": "integer", "minimum": 1, "maximum": 12},
            "speed_m_s": {"type": "number"},
        }, "required": ["husky", "angle_deg", "distance_m", "repetitions"]},
        "impl": tool_repeat_turn_then_drive,
    },
    "drive_to_xy": {
        "tags": ["motion", "essential"],
        "description": (
            "Drive a Husky to an absolute (x, y) coordinate. Closed-loop: it "
            "computes the bearing from live pose, turns, drives, and repeats "
            "until it arrives. PREFER THIS over turn_husky+drive_husky whenever "
            "you know where you want a robot to END UP -- it does the geometry "
            "for you and corrects the open-loop undershoot."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "x": {"type": "number"}, "y": {"type": "number"},
            "tolerance_m": {"type": "number"},
        }, "required": ["husky", "x", "y"]},
        "impl": tool_drive_to_xy,
    },
    "drive_with_fallback": {
        "tags": ["motion", "composite", "essential", "recovery"],
        "description": (
            "Transactional recovery skill: attempt one bounded relative drive; "
            "ONLY if the arena guard refuses it, drive to the operator-supplied "
            "fallback coordinate. Use for explicit 'try X; if refused, do Y' "
            "instructions. Returns both verified stages."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "distance_m": {"type": "number"},
            "fallback_x": {"type": "number"},
            "fallback_y": {"type": "number"},
        }, "required": ["husky", "distance_m", "fallback_x", "fallback_y"]},
        "impl": tool_drive_with_fallback,
    },
    "drive_radial": {
        "tags": ["motion", "essential"],
        "description": (
            "Move a Husky further from (positive delta_r_m) or nearer to "
            "(negative) a centre point, along its own radial direction. USE "
            "THIS for 'fan out', 'spread out', 'move outward', 'close in' -- it "
            "works out each robot's own outward direction from its live "
            "position, so four robots in four quadrants each go the right way. "
            "Also use it for 'point at centre, then drive toward it': pass a "
            "negative delta and never calculate quadrant-specific turn angles."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "delta_r_m": {"type": "number"},
            "center_x": {"type": "number"}, "center_y": {"type": "number"},
        }, "required": ["husky", "delta_r_m"]},
        "impl": tool_drive_radial,
    },
    "ask_operator": {
        "tags": ["essential", "safety"],
        "description": (
            "Ask the operator a clarifying question and STOP. Use this whenever "
            "an instruction does not identify WHICH robot, HOW FAR, or WHERE -- "
            "e.g. 'move it forward a bit'. Nothing moves. Asking is always "
            "correct when the instruction is ambiguous; guessing which robot to "
            "move is not, because the operator cannot tell you guessed."
        ),
        "parameters": {"type": "object", "properties": {
            "question": {"type": "string"},
            "options": {"type": "string"},
        }, "required": ["question"]},
        "impl": tool_ask_operator,
    },
    "find_husky": {
        "tags": ["essential", "telemetry"],
        "description": (
            "Answer a superlative question about the swarm from live pose: which "
            "robot is furthest_from_centre, nearest_to_centre, "
            "furthest_from_point, nearest_to_point, northmost, southmost, "
            "eastmost, westmost. Returns the answer plus the full ranking with "
            "each robot's radius. USE THIS instead of comparing coordinates "
            "yourself."
        ),
        "parameters": {"type": "object", "properties": {
            "criteria": {"type": "string"},
            "x": {"type": "number"}, "y": {"type": "number"},
        }, "required": ["criteria"]},
        "impl": tool_find_husky,
    },
    "find_and_drive": {
        "tags": ["essential", "telemetry", "motion", "composite"],
        "description": (
            "Atomically select one robot from LIVE pose and drive exactly that "
            "robot. USE THIS for 'drive whichever Husky is nearest/furthest/"
            "northmost/...' instead of calling find_husky and copying a name. "
            "criteria supports nearest_to_centre, furthest_from_centre, "
            "nearest_to_point, furthest_from_point, and compass extremes."
        ),
        "parameters": {"type": "object", "properties": {
            "criteria": {"type": "string"},
            "distance_m": {"type": "number"},
            "x": {"type": "number"}, "y": {"type": "number"},
            "speed_m_s": {"type": "number"},
        }, "required": ["criteria", "distance_m"]},
        "impl": tool_find_and_drive,
    },
    "move_swarm_to": {
        "tags": ["essential", "motion", "parallel"],
        "description": (
            "Drive the swarm to a set of target points at once, assigning each "
            "robot to its nearest free target automatically. targets format: "
            "'x1,y1; x2,y2; ...'. USE THIS for formations (a line, a square, a "
            "circle, swapping places): you supply the shape, it works out who "
            "goes where. Avoids two robots being sent to the same point."
        ),
        "parameters": {"type": "object", "properties": {
            "targets": {"type": "string"},
        }, "required": ["targets"]},
        "impl": tool_move_swarm_to,
    },
    "count_huskies": {
        "tags": ["essential", "telemetry"],
        "description": (
            "Count robots matching a spatial predicate from live pose: west "
            "(x<0), east, north, south, within_radius, beyond_radius, all. "
            "Returns the count AND the matching robot names. USE THIS for any "
            "'how many ...' question instead of eyeballing four coordinates."
        ),
        "parameters": {"type": "object", "properties": {
            "region": {"type": "string"},
            "radius_m": {"type": "number"},
            "x": {"type": "number"}, "y": {"type": "number"},
        }, "required": ["region"]},
        "impl": tool_count_huskies,
    },
    "set_husky_velocity": {
        "tags": ["motion"],
        "description": "Set continuous (linear m/s, angular rad/s) velocity until next command.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"}, "linear": {"type": "number"}, "angular": {"type": "number"},
        }, "required": ["husky", "linear", "angular"]},
        "impl": tool_set_husky_velocity,
    },
    "stop_husky": {
        "tags": ["safety", "essential"],
        "description": "Single emergency halt.",
        "parameters": {"type": "object", "properties": {"husky": {"type": "string"}}, "required": ["husky"]},
        "impl": tool_stop_husky,
    },
    "reset_husky_to_home": {
        "tags": ["motion", "supervisor"],
        "description": "Supervisor teleport back to spawn pose.",
        "parameters": {"type": "object", "properties": {"husky": {"type": "string"}}, "required": ["husky"]},
        "impl": tool_reset_husky_to_home,
    },
    "wait_for_husky_idle": {
        "tags": ["sync", "essential"],
        "description": "Block until a Husky's mode is 'idle'. Critical between sequenced legs.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "timeout_s": {"type": "number"},
        }, "required": ["husky"]},
        "impl": tool_wait_for_husky_idle,
    },
    "halt_all": {
        "tags": ["safety", "essential"],
        "description": "Emergency stop all 4 Huskies in parallel. Always available, never loop-gated.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_halt_all,
    },

    # ── Parallel execution ──
    "execute_parallel": {
        "tags": ["parallel", "swarm", "essential"],
        "description": (
            "Dispatch N tool calls at the SAME TIME. Each action is exactly "
            "{tool: '<name>', args: {...}}. For 'all', 'simultaneously', or "
            "'at once', put one action per robot in this single call. Set "
            "halt_on_error=true when the instruction requires fleet "
            "containment if any slot fails. Never issue those motions "
            "sequentially. Cannot be nested. Returns failure_indexes (a slot "
            "that reported a failure), unverified_indexes (a motion slot that "
            "reported NO measured postcondition -- not a success), and "
            "all_completed, which is true only when EVERY slot measured its "
            "own postcondition and it held. Report unverified slots as "
            "unknown; read get_swarm_state before describing them."
        ),
        "parameters": {"type": "object", "properties": {
            "actions": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "tool": {"type": "string"},
                        "args": {"type": "object"},
                    },
                    "required": ["tool"],
                },
            },
            "halt_on_error": {"type": "boolean"},
        }, "required": ["actions"]},
        "impl": tool_execute_parallel,
    },
    "visit_and_return": {
        "tags": ["motion", "waypoint", "composite", "essential"],
        "description": (
            "Atomic checkpoint excursion: snapshot one robot's LIVE current "
            "pose under checkpoint_name BEFORE motion, drive to (x,y), verify "
            "arrival, then return through the saved checkpoint and verify. USE "
            "THIS for inspect/visit/deliver then return/restore instructions."
        ),
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "x": {"type": "number"}, "y": {"type": "number"},
            "checkpoint_name": {"type": "string"},
            "tolerance_m": {"type": "number"},
        }, "required": ["husky", "x", "y", "checkpoint_name"]},
        "impl": tool_visit_and_return,
    },

    # ── Waypoints ──
    "save_waypoint": {
        "tags": ["memory", "spatial"],
        "description": "Save a named (x, y) waypoint. Pass x+y directly OR husky=<id> to snapshot that Husky's current pose. Persistent across sessions.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string"}, "x": {"type": "number"},
            "y": {"type": "number"}, "husky": {"type": "string"},
            "note": {"type": "string"},
        }, "required": ["name"]},
        "impl": tool_save_waypoint,
    },
    "list_waypoints": {
        "tags": ["memory", "spatial"],
        "description": "List every saved waypoint with its coordinates and note.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_waypoints,
    },
    "recall_waypoint": {
        "tags": ["memory", "spatial"],
        "description": "Get one waypoint's coordinates by name.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_recall_waypoint,
    },
    "forget_waypoint": {
        "tags": ["memory", "spatial"],
        "description": "Delete a saved waypoint.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_forget_waypoint,
    },
    "drive_to_waypoint": {
        "tags": ["motion", "spatial", "composite"],
        "description": "Drive one Husky to a saved waypoint. Internally: read pose, compute bearing+distance, turn, wait_for_idle, drive, wait_for_idle. Reports final position error.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"}, "name": {"type": "string"},
            "speed_m_s": {"type": "number"}, "tolerance_m": {"type": "number"},
        }, "required": ["husky", "name"]},
        "impl": tool_drive_to_waypoint,
    },

    # ── Routines ──
    "save_routine": {
        "tags": ["memory", "routine"],
        "description": "Save a named multi-step routine. steps=[{tool, args}, ...]. Persistent across sessions; replay later with run_routine(name).",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string"},
            "steps": {"type": "array", "items": {"type": "object"}},
            "note": {"type": "string"},
        }, "required": ["name", "steps"]},
        "impl": tool_save_routine,
    },
    "list_routines": {
        "tags": ["memory", "routine"],
        "description": "List every saved routine with step counts and notes.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_routines,
    },
    "describe_routine": {
        "tags": ["memory", "routine"],
        "description": "Return a routine's full step list so you can inspect or amend it.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_describe_routine,
    },
    "run_routine": {
        "tags": ["routine", "composite"],
        "description": "Execute a saved routine step by step. Reports elapsed_s, per-step results, and the failed step if stop_on_error and one errored.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string"},
            "stop_on_error": {"type": "boolean"},
        }, "required": ["name"]},
        "impl": tool_run_routine,
    },
    "forget_routine": {
        "tags": ["memory", "routine"],
        "description": "Delete a saved routine.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_forget_routine,
    },

    # ── Free-form memory ──
    "save_memory": {
        "tags": ["memory"],
        "description": "Save a free-form key/value swarm fact. Persistent across sessions. Optional comma-separated tags.",
        "parameters": {"type": "object", "properties": {
            "key": {"type": "string"}, "value": {"type": "string"}, "tags": {"type": "string"},
        }, "required": ["key", "value"]},
        "impl": tool_save_memory,
    },
    "recall_memory": {
        "tags": ["memory"],
        "description": "Get one memory by key.",
        "parameters": {"type": "object", "properties": {"key": {"type": "string"}}, "required": ["key"]},
        "impl": tool_recall_memory,
    },
    "list_memories": {
        "tags": ["memory"],
        "description": "List memories, optionally filtered by key prefix or tag substring.",
        "parameters": {"type": "object", "properties": {
            "prefix": {"type": "string"}, "tag": {"type": "string"},
        }},
        "impl": tool_list_memories,
    },
    "forget_memory": {
        "tags": ["memory"],
        "description": "Delete a memory.",
        "parameters": {"type": "object", "properties": {"key": {"type": "string"}}, "required": ["key"]},
        "impl": tool_forget_memory,
    },

    # ── Outcomes (self-improvement seed) ──
    "report_outcome": {
        "tags": ["learning"],
        "description": "Record the outcome of a task you just ran. success: bool. notes: short string. elapsed_s: optional wall-clock.",
        "parameters": {"type": "object", "properties": {
            "task": {"type": "string"}, "success": {"type": "boolean"},
            "notes": {"type": "string"}, "elapsed_s": {"type": "number"},
        }, "required": ["task", "success"]},
        "impl": tool_report_outcome,
    },
    "recall_past_attempts": {
        "tags": ["learning"],
        "description": "Read past task-outcome records, optionally matching a keyword. Use BEFORE attempting a known-tricky task.",
        "parameters": {"type": "object", "properties": {
            "task_keyword": {"type": "string"}, "limit": {"type": "integer"},
        }},
        "impl": tool_recall_past_attempts,
    },

    # ── Observability ──
    "get_activity_log": {
        "tags": ["observability"],
        "description": "Read the last N tool calls + summarised results. Optional tool name filter.",
        "parameters": {"type": "object", "properties": {
            "last_n": {"type": "integer"}, "tool": {"type": "string"},
        }},
        "impl": tool_get_activity_log,
    },
    "get_swarm_metrics": {
        "tags": ["observability"],
        "description": "Per-Husky cumulative metrics: distance_m, halts, tool_calls.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_get_swarm_metrics,
    },
    "reset_swarm_metrics": {
        "tags": ["observability"],
        "description": "Zero out all per-Husky metrics. Use to start a fresh measurement window.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_reset_swarm_metrics,
    },

    # ── Meta ──
    "list_tools": {
        "tags": ["meta"],
        "description": "Enumerate every tool, optionally filtered by tag.",
        "parameters": {"type": "object", "properties": {"tag": {"type": "string"}}},
        "impl": tool_list_tools,
    },
    "find_tools": {
        "tags": ["meta"],
        "description": "Search tools by keyword. Use when you're not sure which primitive applies.",
        "parameters": {"type": "object", "properties": {
            "query": {"type": "string"}, "limit": {"type": "integer"},
        }, "required": ["query"]},
        "impl": tool_find_tools,
    },
    "invoke_tool": {
        "tags": ["meta"],
        "description": "Dispatch any registered tool by name. Bypasses the manifest -- useful when composing dynamic plans.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string"}, "args": {"type": "object"},
        }, "required": ["name"]},
        "impl": tool_invoke_tool,
    },
}


# ─── Dispatcher (with loop budget gate) ────────────────────────────

def _coerce_args(spec: Dict[str, Any], args: Dict[str, Any]) -> Dict[str, Any]:
    """Bend LLM-supplied argument types to the declared schema.

    Models routinely send a JSON list for a field named "tags" even when the
    schema says string -- which then reaches sqlite3 as a list and raises
    InterfaceError. Rather than special-casing one tool, coerce any value whose
    declared type is "string" into one.
    """
    props = ((spec.get("parameters") or {}).get("properties") or {})
    if not props or not isinstance(args, dict):
        return args
    out = dict(args)
    for key, value in args.items():
        declared = (props.get(key) or {}).get("type")
        if declared != "string" or isinstance(value, str) or value is None:
            continue
        if isinstance(value, (list, tuple)):
            out[key] = ", ".join(str(v) for v in value)
        elif isinstance(value, (int, float, bool)):
            out[key] = str(value)
        elif isinstance(value, dict):
            out[key] = json.dumps(value)
    return out


def dispatch(tool_name: str, args: Dict[str, Any], gate: bool = True) -> Dict[str, Any]:
    spec = TOOLS.get(tool_name)
    if spec is None:
        return {"error": f"unknown tool: {tool_name}",
                "hint": "call list_tools or find_tools to see available tools",
                "known_tools_sample": sorted(TOOLS.keys())[:10]}
    if gate:
        refusal = _check_tool_loop_budget(tool_name)
        if refusal is not None:
            return refusal
        _record_tool_call(tool_name)
    # Control-ownership gate. Opt-in (HUSKY_SWARM_REQUIRE_OWNER=1) because it
    # changes behaviour for existing callers: with it on, motion is refused
    # unless an operator has claimed control via POST /control. Without it we
    # keep the previous last-write-wins behaviour, which is fine for a single
    # operator and surprising with a chat window open alongside the agent.
    if CONTROL_OWNER_REQUIRED and "motion" in (spec.get("tags") or []):
        with _CONTROL_LOCK:
            owner = _CONTROL_OWNER
        if not owner:
            return {"error": "no_control_owner",
                    "hint": "Motion requires an operator to claim control first "
                            "(POST /control {\"owner\": \"...\"}). Report this "
                            "rather than retrying."}

    args = _coerce_args(spec, args)
    try:
        result = spec["impl"](**args)
    except TypeError as e:
        return {"error": f"bad args: {e}"}
    except Exception as e:
        # Never swallow this. A bare "tool_execution_failed" told the agent
        # nothing, so it assumed success and reported a save that never
        # happened -- the tool layer lying to the model is as bad as the
        # model lying to the operator.
        detail = f"{type(e).__name__}: {e}"
        print(f"[husky_swarm] tool {tool_name} failed: {detail}", flush=True)
        traceback.print_exc()
        return {"error": "tool_execution_failed", "detail": detail[:300],
                "hint": "This tool did NOT run. Do not report its effect as done."}
    # Per-Husky tool-call counter for the metrics surface.
    husky = args.get("husky") if isinstance(args, dict) else None
    if husky and husky in _METRICS:
        with _METRICS_LOCK:
            _METRICS[husky]["tool_calls"] = _METRICS[husky].get("tool_calls", 0) + 1
    _log_activity(tool_name, args, result)
    return result


# ─── Sim relaunch (operator/UI surface) ────────────────────────────
# POST /relaunch-sim: if the bridges are unreachable (the sim died), start
# the swarm world again — WORLD ONLY, detached, never the agent. SAFETY:
# this surface only ever STARTS a sim; it never stops, kills, or reloads
# anything. Pairs with the runner's --keep-agent mode
# (scripts/dev/omnisim_run_agent.py): the runner keeps this agent alive
# when the sim exits, and the UI brings the world back from here.

_RELAUNCH_LOCK = threading.Lock()
_RELAUNCH_STATE: Dict[str, Any] = {"proc": None, "t0": 0.0}

_PROBE_LOCK = threading.Lock()
_PROBE_CACHE: Dict[str, Any] = {"t": 0.0, "result": {}}
_PROBE_CACHE_TTL_S = 1.5


def _probe_bridge(url: str, timeout_s: float = 1.0) -> bool:
    """One reachability probe against a bridge's GET /state."""
    try:
        with urllib.request.urlopen(url.rstrip("/") + "/state", timeout=timeout_s) as r:
            return 200 <= r.status < 300
    except Exception:
        return False


def _probe_bridges(timeout_s: float = 1.0) -> Dict[str, bool]:
    """Probe every bridge in parallel. Cached ~1.5 s so a UI polling
    /health does not hammer four sockets per poll."""
    with _PROBE_LOCK:
        if (time.time() - _PROBE_CACHE["t"] < _PROBE_CACHE_TTL_S
                and _PROBE_CACHE["result"]):
            return dict(_PROBE_CACHE["result"])
    out: Dict[str, bool] = {}
    threads = []
    for name, url in BRIDGES.items():
        t = threading.Thread(
            target=lambda n=name, u=url: out.__setitem__(n, _probe_bridge(u, timeout_s)),
            daemon=True)
        t.start()
        threads.append(t)
    for t in threads:
        t.join(timeout_s + 1.0)
    for name in BRIDGES:
        out.setdefault(name, False)
    with _PROBE_LOCK:
        _PROBE_CACHE["t"] = time.time()
        _PROBE_CACHE["result"] = dict(out)
    return out


def _relaunch_inflight(sim_up: bool = False) -> Dict[str, Any]:
    """UI-facing view of the relaunch guard, for /health.

    ``in_flight`` means "a relaunched sim is still coming up": once the
    bridges answer (``sim_up``), the relaunch is complete even though the
    detached sim process is, of course, still alive."""
    with _RELAUNCH_LOCK:
        proc = _RELAUNCH_STATE.get("proc")
        alive = proc is not None and proc.poll() is None
        return {"in_flight": alive and not sim_up,
                "pid": proc.pid if alive else None}


def _sim_world_command() -> Tuple[List[str], Dict[str, str]]:
    """Derive the exact headless sim command + env THE SAME WAY the runner
    does — by importing scripts/dev/omnisim_run_agent.py and asking it
    (binary discovery, headless flags, OMNISIM_HOME env included)."""
    runner_dir = _REPO_ROOT / "scripts" / "dev"
    if str(runner_dir) not in sys.path:
        sys.path.insert(0, str(runner_dir))
    import omnisim_run_agent as _runner  # noqa: E402  (lazy: only on relaunch)
    spec = _runner.AGENTS.get("husky_swarm")
    if spec is None:
        raise RuntimeError("husky_swarm is not in the runner's agent registry")
    return _runner.world_command(spec, headless=True), _runner.omnisim_env()


def _relaunch_sim() -> Tuple[int, Dict[str, Any]]:
    """Implements POST /relaunch-sim. Returns (http_status, payload)."""
    # Re-probe first: if any bridge answers, the sim is up — do nothing.
    probe_url = BRIDGES.get("husky_ne") or next(iter(BRIDGES.values()), "")
    if probe_url and _probe_bridge(probe_url, timeout_s=1.5):
        detail = "The sim is already up: a bridge answered its /state probe."
        print(f"  [relaunch-sim] no-op: {detail}")
        return 200, {"ok": True, "status": "already-running", "detail": detail}
    with _RELAUNCH_LOCK:
        proc = _RELAUNCH_STATE.get("proc")
        if proc is not None and proc.poll() is None:
            age = time.time() - _RELAUNCH_STATE["t0"]
            detail = (f"A relaunch started {age:.0f}s ago is still coming up "
                      f"(pid {proc.pid}); not starting a second sim.")
            print(f"  [relaunch-sim] in-flight: {detail}")
            return 200, {"ok": True, "status": "launching", "detail": detail}
        try:
            argv, env = _sim_world_command()
        except Exception as exc:
            print(f"  [relaunch-sim] ERROR deriving the sim command: {exc}")
            return 500, {"ok": False,
                         "error": f"could not derive the sim command: {exc}"}
        try:
            kwargs: Dict[str, Any] = {
                "env": env,
                "cwd": str(_REPO_ROOT),
                "stdin": subprocess.DEVNULL,
                "stdout": subprocess.DEVNULL,
                "stderr": subprocess.DEVNULL,
            }
            if os.name == "nt":
                kwargs["creationflags"] = (
                    subprocess.CREATE_NEW_PROCESS_GROUP
                    | subprocess.DETACHED_PROCESS)  # type: ignore[attr-defined]
            else:
                kwargs["start_new_session"] = True
            proc = subprocess.Popen(argv, **kwargs)
        except Exception as exc:
            print(f"  [relaunch-sim] ERROR spawning the sim: {exc}")
            return 500, {"ok": False, "error": f"failed to spawn the sim: {exc}"}
        _RELAUNCH_STATE["proc"] = proc
        _RELAUNCH_STATE["t0"] = time.time()
    detail = (f"Bridges were unreachable, so the swarm world is launching "
              f"detached (pid {proc.pid}); bridges typically answer within "
              f"30-60 s.")
    print(f"  [relaunch-sim] {detail}")
    print(f"  [relaunch-sim] command: {' '.join(argv)}")
    return 200, {"ok": True, "status": "launching", "detail": detail}


# ─── HTTP server (platform tool callback) ──────────────────────────

def _make_handler() -> Any:
    # The swarm operator UI dev server is built against this surface on
    # localhost:5174; trust it by default so /tool and /relaunch-sim work
    # from the browser without OMNISIM_ALLOWED_ORIGINS in every shell.
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
            self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token")

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
                # Operator surface. Deliberately NOT a registered tool: the
                # model cannot see it, call it, or clear it.
                self._write_json(200, estop_state()); return
            if self.path == "/activity":
                body = json.dumps({"entries": ACTIVITY_LOG[-100:]}, default=str).encode()
            elif self.path == "/health":
                # bridges_reachable/sim_up let the UI distinguish
                # agent-up-bridges-down (sim dead: offer /relaunch-sim)
                # from everything-up.
                bridges_reachable = _probe_bridges()
                body = json.dumps({
                    "ok": True,
                    "bridges": BRIDGES,
                    "bridges_reachable": bridges_reachable,
                    "sim_up": any(bridges_reachable.values()),
                    "relaunch": _relaunch_inflight(
                        sim_up=any(bridges_reachable.values())),
                    "tool_count": len(TOOLS),
                    "db_path": str(DB_PATH),
                    "arena_bound_m": ARENA_BOUND_M,
                    "estop": estop_state(),
                    "bridge_token": bool(BRIDGE_TOKEN),
                    "control_owner": _CONTROL_OWNER or None,
                }, default=str).encode()
            elif self.path == "/metrics":
                with _METRICS_LOCK:
                    body = json.dumps({"metrics": dict(_METRICS)}, default=str).encode()
            else:
                self.send_error(404); return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors(); self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            try:
                self._guard()
                if self.path in ("/estop", "/control"):
                    body = read_json(self, allow_empty=True) or {}
                    if self.path == "/estop":
                        action = str(body.get("action") or "engage").strip().lower()
                        reason = str(body.get("reason") or "operator").strip()
                        out = estop_clear() if action == "clear" else estop_engage(reason)
                        self._write_json(200, out)
                        return
                    global _CONTROL_OWNER
                    with _CONTROL_LOCK:
                        if str(body.get("action") or "").lower() == "release":
                            _CONTROL_OWNER = ""
                        else:
                            _CONTROL_OWNER = str(body.get("owner") or "").strip()
                        self._write_json(200, {"owner": _CONTROL_OWNER or None})
                        return
                if self.path == "/relaunch-sim":
                    # Operator/UI surface (same guard as /tool). Only ever
                    # STARTS the sim; see _relaunch_sim.
                    read_json(self, allow_empty=True)  # accept empty or {}
                    status, payload = _relaunch_sim()
                    self._write_json(status, payload)
                    return
                if self.path != "/tool":
                    self._write_json(404, error_envelope("not_found", "Unknown callback path.")); return
                data = read_json(self, allow_empty=False)
            except RequestError as exc:
                self._write_json(exc.status, error_envelope(exc.code, exc.message, exc.details))
                return
            tool_name = data.pop("tool", "")
            if not isinstance(tool_name, str) or not tool_name.strip():
                self._write_json(400, error_envelope("missing_field", "Field 'tool' is required.", {"field": "tool"}))
                return
            print(f"  [TOOL] {tool_name}({_short(data)})")
            with action_lock:
                result = dispatch(tool_name, data)
            self._write_json(200, {"status": "ok", "tool": tool_name, "result": result})

    return Handler


def _short(d: Any, n: int = 120) -> str:
    s = json.dumps(d, default=str)
    return s if len(s) <= n else s[:n - 3] + "..."


def start_tool_server() -> int:
    token = bearer_token()
    validate_bind_host(SWARM_HOST, token)
    try:
        server = http.server.ThreadingHTTPServer((SWARM_HOST, SWARM_PORT), _make_handler())
    except OSError as e:
        print(f"  [WARN] port {SWARM_PORT} taken ({e}); falling back to random.")
        server = http.server.ThreadingHTTPServer((SWARM_HOST, 0), _make_handler())
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return port


# ─── Profile push ──────────────────────────────────────────────────

def load_profile() -> Dict[str, Any]:
    with open(PROFILE_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _advertised_tools() -> List[str]:
    """Which tools go into the pushed profile.

    Every registered tool stays dispatchable — this only controls what is
    ADVERTISED. The default ``performance`` tier publishes the compact
    operating surface and the model reaches everything else through
    find_tools + invoke_tool. ``full`` remains available for inspection and
    compatibility experiments.

    Why: the full manifest is ~22k tokens of prompt before the operator has
    said anything. Benchmarked 2026-07-22, llama3.3:70b took one look at it and
    refused outright ("Your function definitions are not comprehensive enough
    for this task"), and qwen2.5:14b managed 2/4 against 10/10 on the arm chat
    demos — which advertise ~5 tools. Tool count was the biggest uncontrolled
    variable between those two results.
    """
    tier = os.environ.get("HUSKY_SWARM_TOOL_TIER", "performance").strip().lower()
    if tier == "full":
        return list(TOOLS)
    if tier == "performance":
        # High-frequency operating surface. Long-tail tools remain reachable
        # through find_tools + invoke_tool, so reducing schema entropy does not
        # remove capability. The order is intentional: observe, act, compose,
        # stop, then discover.
        preferred = [
            "get_swarm_state", "get_husky_status",
            "drive_husky", "turn_husky", "turn_then_drive", "drive_to_xy",
            "repeat_turn_then_drive",
            "drive_with_fallback", "visit_and_return",
            "find_husky", "find_and_drive", "drive_radial", "move_swarm_to",
            "execute_parallel", "stop_husky", "halt_all", "ask_operator",
            "list_tools", "find_tools", "invoke_tool",
        ]
        return [name for name in preferred if name in TOOLS]
    keep = [n for n, s in TOOLS.items()
            if "essential" in (s.get("tags") or []) or "meta" in (s.get("tags") or [])]
    return keep or list(TOOLS)



# ─── Rule-set variants (A/B) ────────────────────────────────────────
#
# The behavioural rules grew to 21 numbered directives, ~6.2k of a 10.2k-char
# mainTask -- 60% of the prompt before the operator says anything. That helped
# gemini-3.1-flash-lite (expert 3/6 -> 5/6) and coincided with grok-4.5 getting
# WORSE (3/6 -> 2/6). Two explanations: model variance, or the rule bulk
# crowding out the task. HUSKY_SWARM_RULE_SET=core swaps in a condensed set
# covering the same ground so the two can be told apart.

CORE_RULES = """--- BEHAVIOURAL RULES (core) ---

1. EVIDENCE, NOT MEMORY. Never state a position or outcome you have not just
   read from a tool result in this turn. `idle` means "not moving" -- it is NOT
   evidence that a move happened; a robot that never got a command is idle too.
   Only report success after a fresh read shows the change you intended.

2. LET TOOLS DO THE MATHS. Use drive_to_xy / drive_radial for "where should it
   end up", find_husky for "which robot is most/least...", count_huskies for
   "how many...", move_swarm_to for formations (give it the points, it assigns
   robots). Computing headings or comparing coordinates yourself is how you
   send robots the wrong way or move none at all.

3. ASK WHEN THE ORDER IS INCOMPLETE. If it does not say WHICH robot, HOW FAR,
   or WHERE, call ask_operator and stop. Do not pick for them -- a wrong guess
   is indistinguishable from a followed instruction.

4. A REFUSAL IS NOT A NEGOTIATION. If a guard refuses a move as out of bounds,
   report the limit and ask. Never substitute a smaller version of an
   impossible order.

5. SAVE STATE BEFORE YOU DESTROY IT. If the task mentions where something
   "started", or asks you to restore anything, save_waypoint FIRST. Once a
   robot leaves a spot that coordinate is gone.

6. FINISH EVERY CLAUSE. "Do A, then once it arrives do B" is not done when A
   is done. Re-read the instruction before replying and check each part
   actually happened.

7. REPORT PARTIAL FAILURE AS PARTIAL. If some robots succeeded and others did
   not, say exactly which and why. Accurate numbers attached to a false
   "all done" is still a lie.

8. STOP means halt_all first, then ask.
"""

PERFORMANCE_MAIN_TASK = """Coordinate four OmniSim robots: husky_ne, husky_nw,
husky_se, husky_sw. Execute with tools; report observed results. Bounds: +/-7 m.

ACTION ROUTER -- apply the first matching route:
1. Current pose/status is READ-ONLY: get_husky_status for one or get_swarm_state
   for several. Never answer from spawn coordinates or memory.
2. Stop/halt/freeze: call stop_husky for one or halt_all for the fleet FIRST.
3. "Turn, then drive": turn_then_drive once.
4. "Repeat N times", square, or polygon on ONE robot:
   repeat_turn_then_drive once with repetitions=N. Never execute_parallel
   multiple actions for the same robot; dependent legs must be sequential.
5. "Visit then return/restore": visit_and_return. "Try X; if refused, do Y":
   drive_with_fallback. These skills own ordering and verification.
6. Explicit unit-agent work ("ask each unit", "delegate", "unit agent"):
   native delegate_to_agent to HuskyNE/NW/SE/SW. Status: "Call get_status now
   and copy pose_text exactly." Use separate native calls, NEVER
   execute_parallel. After reports, delegate action again. On failure stop;
   NEVER substitute coordinator motion.
7. "Point at centre and move toward/away from it", "close in", or "fan out":
   drive_radial (negative delta=toward, positive=away). For a simultaneous
   fleet request, put one drive_radial per robot inside execute_parallel. NEVER
   calculate quadrant-specific turn angles.
8. An absolute destination: call drive_to_xy. A relative forward/back move:
   call drive_husky.
9. "Whichever is nearest/furthest/...": find_and_drive.
10. "All", "simultaneously", or "at once": call execute_parallel ONCE with one
    action per DISTINCT robot. If the operator says halt on any failure, set
    halt_on_error=true. Never simulate parallelism with sequential calls.
11. For an uncommon capability, call find_tools, then invoke_tool.

EXECUTION CONTRACT:
- MEMORY IS HISTORICAL, NEVER LIVE STATE. Only THIS mission's tool results
  establish bridge/e-stop/motion state. Otherwise attempt once; guards enforce.
- A requested physical action is not done unless a mutating tool ran.
- `accepted` is not completion. Trust verified `completed`/final_pose; otherwise
  report failure without blind retry.
- Do not move robots that were not named or selected by live telemetry.
- Refuse out-of-bounds/unbounded motion and surface the +/-7 m limit. Never
  replace a refused order with a smaller move.
- Keep the final report terse: what ran, measured result, and any failure.
"""


def _apply_rule_set(main_task: str) -> str:
    """Swap the numbered rule block for the condensed one when asked."""
    if os.environ.get("HUSKY_SWARM_RULE_SET", "").strip().lower() != "core":
        return main_task
    start = main_task.find("--- BEHAVIOURAL RULES ---")
    end = main_task.find("Good prompts you should be able to handle")
    if start < 0 or end < 0 or end <= start:
        return main_task
    return main_task[:start] + CORE_RULES + "\n" + main_task[end:]


def _main_task_for_runtime(base_main_task: str) -> str:
    """Select the production prompt profile.

    The historical extended brief is useful as documentation and for A/B
    comparisons, but advertising the whole registry plus twenty-one
    overlapping rules makes the model spend attention recovering the control
    policy on every turn. Production defaults to the compact router; operators
    can set HUSKY_SWARM_PROMPT_PROFILE=extended to restore the old prompt
    verbatim.
    """
    profile = os.environ.get(
        "HUSKY_SWARM_PROMPT_PROFILE", "performance").strip().lower()
    if profile == "extended":
        return _apply_rule_set(base_main_task)
    return PERFORMANCE_MAIN_TASK


def _team_agent_names(bridges: Dict[str, str]) -> List[str]:
    """Unit-agent names for the huskies in a bridges map: 'husky_ne' ->
    'HuskyNE'. One dedicated unit agent per husky present (see
    husky_unit_agent.py); this is the platform delegation roster."""
    out: List[str] = []
    for key in sorted(bridges):
        hid = key[len("husky_"):] if key.startswith("husky_") else key
        out.append("Husky" + hid.upper())
    return out


# Prompt/registry drift is a SHIPPED bug class here, not a hypothetical: the
# profile claimed 28 tools when 40 were registered, then 40 when 45 were, and
# the push-time fix was a literal string replace that only matched the exact
# stale sentence -- so the very next drift sailed through. Both halves are now
# derived from TOOLS: the count is rewritten by pattern, and the layer
# enumeration is AUDITED against the registry with anything missing appended,
# so the live prompt can never advertise less than the agent actually holds.
_TOOL_COUNT_CLAIM_RE = re.compile(
    r"\b\d+(?= (?:registered )?tools\b| primitives\b)")


def _retarget_tool_count_claims(text: str) -> str:
    """Rewrite every numeric tool-count claim to the registry's real size."""
    return _TOOL_COUNT_CLAIM_RE.sub(str(len(TOOLS)), text)


def tools_missing_from_prompt(text: str) -> List[str]:
    """Registered tools the prompt never names. Empty is the healthy state."""
    return sorted(name for name in TOOLS if name not in (text or ""))


def build_settings(base: Dict[str, Any], tool_callback_url: str) -> Dict[str, Any]:
    advertised = _advertised_tools()
    tool_defs = [
        {"name": n, "description": TOOLS[n]["description"], "parameters": TOOLS[n]["parameters"]}
        for n in advertised
    ]
    if len(advertised) < len(TOOLS):
        print(f"  CURATED tool surface: advertising {len(advertised)} of {len(TOOLS)} "
              f"(rest reachable via find_tools/invoke_tool)")
    settings = dict(base)
    if settings.get("mainTask"):
        original = _retarget_tool_count_claims(settings["mainTask"])
        missing = tools_missing_from_prompt(original)
        if missing:
            print(f"  PROMPT DRIFT: {len(missing)} registered tool(s) missing "
                  f"from the layer enumeration: {', '.join(missing)}")
            print("               appending them so the live prompt is not "
                  "short. Fix profile.json.")
            original += (
                "\n\nALSO REGISTERED (not yet placed in a layer above): "
                + ", ".join(missing)
                + ". Call list_tools or find_tools for their parameters.")
        settings["mainTask"] = _main_task_for_runtime(original)
        if settings["mainTask"] != original:
            print(f"  PROMPT PROFILE: performance ({len(original)} -> "
                  f"{len(settings['mainTask'])} chars)")
    # Persona-first, same convention as the arm bridge: the character leads,
    # then the operational brief + rules. The platform's chat handler reads
    # `mainTask` as the system instruction, so the persona must be baked into
    # it -- a sibling `persona` key alone would never reach the model.
    # Team of per-husky unit agents (husky_unit_agent.py), one per bridge.
    # The platform's delegation surface reads settings.team; the sentence in
    # mainTask tells the model the roster exists and what it is for.
    team = _team_agent_names(BRIDGES)
    settings["team"] = team
    if settings.get("mainTask") and team:
        settings["mainTask"] += (
            "\n\nTEAM: you command dedicated unit agents via delegate_to_agent -- "
            + ", ".join(team)
            + " -- each owning exactly one Husky (HuskyNE owns husky_ne, and so "
              "on); delegate single-robot status checks and errands to the "
              "owning unit agent by name, and keep fleet-level coordination "
              "yourself.")
    persona = (settings.get("persona") or "").strip()
    if persona and settings.get("mainTask"):
        settings["mainTask"] = persona + "\n\n" + settings["mainTask"]
        print(f"  Persona: baked into mainTask ({len(persona)} chars)")
    settings["availableTools"] = ", ".join(t["name"] for t in tool_defs)
    settings["availableToolDetails"] = tool_defs
    settings["allowToolUse"] = True
    settings["toolCallbackUrl"] = tool_callback_url
    settings.setdefault("engine", "g2-engine")
    return settings


def ensure_profile(client: Any, agent_name: str, settings: Dict[str, Any]) -> str:
    profiles = client.list_profiles()
    existing = next((p for p in profiles if (p.get("name") or "").lower() == agent_name.lower()), None)
    if existing:
        pid = existing["id"]
        client.update_profile(pid, name=agent_name, settings=settings)
        print(f"  Profile updated: {agent_name} (id={pid})")
        return pid
    result = client.create_profile(agent_name, settings=settings)
    pid = result.get("id", "")
    print(f"  Profile created: {agent_name} (id={pid})")
    return pid


# ─── Presence heartbeat ─────────────────────────────────────────────
#
# The profile push above only makes the agent LISTED; the /agents roster
# decides ONLINE from heartbeats. Same wire contract as the bridge relay
# (packages/omnisim-bridges/src/omnisim_bridges/relay.py, _presence_loop /
# _post_presence): POST /api/relay-heartbeat with {agentName, kind, detail}
# where detail carries the interval hint the platform sizes its staleness
# window from. Auth is Bearer OMNI_KEY. Presence is telemetry: it must
# never take the agent down and never spam the log -- a missed beat
# already reads as "offline", which is the honest signal.

PRESENCE_INTERVAL_S = float(os.environ.get("HUSKY_SWARM_PRESENCE_INTERVAL_S", "5"))


def start_presence_heartbeat(agent_name: str, omni_key: str,
                             engine: str, endpoint: str = "") -> threading.Event:
    """Start the daemon heartbeat thread; returns the stop Event."""
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
                # Silent retry on the next beat -- see the header comment.
                pass
            stop.wait(PRESENCE_INTERVAL_S)

    threading.Thread(target=_loop, name="omnilink-presence", daemon=True).start()
    return stop


def main() -> int:
    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set. Get one at https://www.omnilink-agents.com/key\n"
              "       or run: python -m omnisim key")
        return 1

    _store_init()
    print(f"  Persistent store: {DB_PATH}")

    profile = load_profile()
    agent_name = profile["name"]
    base_settings = profile["settings"]

    port = start_tool_server()
    tool_callback = f"http://127.0.0.1:{port}/tool"
    print(f"  Tool callback: {tool_callback}")
    print(f"  Configured Huskies: {BRIDGES}")
    print(f"  {len(TOOLS)} tools registered")
    if BRIDGE_TOKEN:
        print("  Bridge auth: ON (OMNISIM_BRIDGE_TOKEN set)")
    else:
        print("  [SECURITY] Bridge auth OFF -- OMNISIM_BRIDGE_TOKEN is unset, so")
        print("             check_authorization() is a no-op and ANY local process")
        print("             (including any browser tab on a permitted origin) can")
        print("             POST motion commands straight to the robot bridges.")
        print("             Set it on BOTH the world and this agent to close that.")
    print(f"  E-stop: POST /estop  (operator-only; not a model-callable tool)")
    print(f"  Sim relaunch: POST /relaunch-sim  (starts the world when the "
          f"bridges are down; never kills anything)")

    client = OmniLinkClient(omni_key=omni_key, base_url=BASE_URL, timeout=30)
    settings = build_settings(base_settings, tool_callback)
    ensure_profile(client, agent_name, settings)

    presence_stop = start_presence_heartbeat(
        agent_name, omni_key,
        engine=str(settings.get("engine", "")),
        endpoint=tool_callback,
    )
    print(f"  Presence heartbeat: every {PRESENCE_INTERVAL_S:g}s -> "
          f"{BASE_URL.rstrip('/')}/api/relay-heartbeat")

    print(f"  Ready. Open https://omnilink-agents.com and pick {agent_name!r}.")
    print(f"  Ctrl-C to exit.")
    try:
        while True:
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("  HuskySwarm shutting down.")
        presence_stop.set()
        return 0


if __name__ == "__main__":
    sys.exit(main())
