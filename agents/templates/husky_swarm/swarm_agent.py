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
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

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
POLL_INTERVAL_S = float(os.environ.get("HUSKY_SWARM_POLL_INTERVAL_S", "3"))
BRIDGE_TIMEOUT_S = float(os.environ.get("HUSKY_SWARM_BRIDGE_TIMEOUT_S", "10"))

# Arena bounds for the husky_swarm world. Used by the safety guard.
ARENA_BOUND_M = float(os.environ.get("HUSKY_SWARM_ARENA_BOUND_M", "7.0"))

# Tool-loop budget: bound run-away tool firing patterns. Same idea as
# a rate-limit gate. Window is a rolling 30 seconds.
TOOL_BUDGET_WINDOW_S = 30.0
TOOL_BUDGET_TOTAL = int(os.environ.get("HUSKY_SWARM_TOOL_BUDGET", "12"))
TOOL_BUDGET_PER_TOOL = int(os.environ.get("HUSKY_SWARM_TOOL_BUDGET_PER", "6"))

# Persistent store. Override location via HUSKY_SWARM_DB.
DB_PATH = Path(
    os.environ.get("HUSKY_SWARM_DB")
    or (Path.home() / ".omnisim" / "husky_swarm.sqlite3")
).expanduser()


def _parse_bridges_env() -> Dict[str, str]:
    spec = os.environ.get("HUSKY_SWARM_BRIDGES", "").strip()
    if not spec:
        return {
            "husky_ne": "http://127.0.0.1:8765",
            "husky_nw": "http://127.0.0.1:8766",
            "husky_se": "http://127.0.0.1:8767",
            "husky_sw": "http://127.0.0.1:8768",
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

def _bridge_post(husky: str, path: str, payload: Optional[Dict[str, Any]] = None,
                 timeout: Optional[float] = None) -> Dict[str, Any]:
    url = BRIDGES.get(husky)
    if not url:
        return {"error": f"unknown husky {husky!r}", "known_huskies": sorted(BRIDGES.keys())}
    full = f"{url}/{path.lstrip('/')}"
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(full, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
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
    Uses the most recent get_robot_state read; falls back to spawn if
    unknown. Conservative: assumes the worst-case heading."""
    try:
        state = _bridge_post(husky, "get_robot_state", timeout=2)
        x = float(state.get("x", SWARM_QUADRANT_SPAWN.get(husky, (0, 0))[0]))
        y = float(state.get("y", SWARM_QUADRANT_SPAWN.get(husky, (0, 0))[1]))
        yaw = float(state.get("yaw", 0.0))
    except Exception:
        return None  # if we can't read state, let it through; bridge will clamp
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
                f"That distance ({distance_m} m) would push {husky} past the "
                f"+/-{ARENA_BOUND_M} m arena bound. Shorten the move or "
                "rotate first."
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
    """Straight-line motion in metres along current heading. Arena-guarded."""
    if not husky:
        return {"error": "husky is required"}
    blocked = _guard_drive_distance(husky, float(distance_m))
    if blocked is not None:
        return blocked
    payload: Dict[str, Any] = {"distance": float(distance_m)}
    if speed_m_s is not None:
        payload["speed"] = float(speed_m_s)
    return _bridge_post(husky, "drive_forward", payload)


def tool_turn_husky(husky: str = "", angle_deg: float = 0.0, **_: Any) -> Dict[str, Any]:
    """Rotation in degrees. Positive = counter-clockwise."""
    if not husky:
        return {"error": "husky is required"}
    return _bridge_post(husky, "turn", {"angle": math.radians(float(angle_deg))})


def tool_set_husky_velocity(husky: str = "", linear: float = 0.0,
                            angular: float = 0.0, **_: Any) -> Dict[str, Any]:
    if not husky:
        return {"error": "husky is required"}
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

def tool_execute_parallel(actions: Optional[List[Dict[str, Any]]] = None, **_: Any) -> Dict[str, Any]:
    """Dispatch N tool calls in parallel. Returns per-action results
    keyed by index. The agent's primary way to issue swarm-wide
    commands in one network round-trip instead of N sequential ones."""
    if not actions or not isinstance(actions, list):
        return {"error": "actions must be a non-empty list of {tool, args}"}
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
    for t in threads: t.join(timeout=BRIDGE_TIMEOUT_S * 2)
    return {"results": results, "count": len(actions), "timestamp": time.time()}


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
        "tags": ["bridge", "telemetry"],
        "description": "Single-Husky state read.",
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
        "description": "Straight-line motion in metres along current heading. Arena-bound-guarded. Positive=forward.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "distance_m": {"type": "number"},
            "speed_m_s": {"type": "number"},
        }, "required": ["husky", "distance_m"]},
        "impl": tool_drive_husky,
    },
    "turn_husky": {
        "tags": ["motion", "essential"],
        "description": "Rotation in degrees. Positive=counter-clockwise.",
        "parameters": {"type": "object", "properties": {
            "husky": {"type": "string"},
            "angle_deg": {"type": "number"},
        }, "required": ["husky", "angle_deg"]},
        "impl": tool_turn_husky,
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
            "Dispatch N tool calls in parallel. Each action is "
            "{tool, args}. Returns per-slot results. Use this -- not "
            "four sequential calls -- to issue swarm-wide commands. "
            "Cannot be nested."
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
        }, "required": ["actions"]},
        "impl": tool_execute_parallel,
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
    try:
        result = spec["impl"](**args)
    except TypeError as e:
        return {"error": f"bad args: {e}"}
    except Exception as e:
        return {"error": f"tool crashed: {e.__class__.__name__}: {e}"}
    # Per-Husky tool-call counter for the metrics surface.
    husky = args.get("husky") if isinstance(args, dict) else None
    if husky and husky in _METRICS:
        with _METRICS_LOCK:
            _METRICS[husky]["tool_calls"] = _METRICS[husky].get("tool_calls", 0) + 1
    _log_activity(tool_name, args, result)
    return result


# ─── HTTP server (platform tool callback) ──────────────────────────

def _make_handler() -> Any:
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_: Any) -> None: pass

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self) -> None:
            self.send_response(200); self._cors(); self.end_headers()

        def do_GET(self) -> None:
            if self.path == "/activity":
                body = json.dumps({"entries": ACTIVITY_LOG[-100:]}, default=str).encode()
            elif self.path == "/health":
                body = json.dumps({
                    "ok": True,
                    "bridges": BRIDGES,
                    "tool_count": len(TOOLS),
                    "db_path": str(DB_PATH),
                    "arena_bound_m": ARENA_BOUND_M,
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
            if self.path != "/tool":
                self.send_error(404); return
            n = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(n).decode("utf-8", errors="replace")
            try:
                data = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                data = {}
            tool_name = data.pop("tool", "")
            print(f"  [TOOL] {tool_name}({_short(data)})")
            result = dispatch(tool_name, data)
            body = json.dumps({"status": "ok", "tool": tool_name, "result": result}, default=str).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors(); self.end_headers()
            self.wfile.write(body)

    return Handler


def _short(d: Any, n: int = 120) -> str:
    s = json.dumps(d, default=str)
    return s if len(s) <= n else s[:n - 3] + "..."


def start_tool_server() -> int:
    try:
        server = http.server.ThreadingHTTPServer(("0.0.0.0", SWARM_PORT), _make_handler())
    except OSError as e:
        print(f"  [WARN] port {SWARM_PORT} taken ({e}); falling back to random.")
        server = http.server.ThreadingHTTPServer(("0.0.0.0", 0), _make_handler())
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return port


# ─── Profile push ──────────────────────────────────────────────────

def load_profile() -> Dict[str, Any]:
    with open(PROFILE_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def build_settings(base: Dict[str, Any], tool_callback_url: str) -> Dict[str, Any]:
    tool_defs = [
        {"name": n, "description": s["description"], "parameters": s["parameters"]}
        for n, s in TOOLS.items()
    ]
    settings = dict(base)
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


def main() -> int:
    omni_key = os.environ.get("OMNI_KEY", "").strip()
    if not omni_key:
        print("ERROR: OMNI_KEY not set. Get one from https://omnilink-agents.com/account.")
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

    client = OmniLinkClient(omni_key=omni_key, base_url=BASE_URL, timeout=30)
    settings = build_settings(base_settings, tool_callback)
    ensure_profile(client, agent_name, settings)

    print(f"  Ready. Open https://omnilink-agents.com and pick {agent_name!r}.")
    print(f"  Ctrl-C to exit.")
    try:
        while True:
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        print("  HuskySwarm shutting down.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
