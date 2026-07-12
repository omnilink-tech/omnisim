# Copyright 2026 OmniLink
# Licensed under the Apache License, Version 2.0.

"""MissionControl -- OmniLink-driven dispatch coordinator for OmniSim's
6-Husky logistics campus.

Runs as a separate Python process. Pushes a "MissionControl" profile
to omnilink-agents.com, hosts a /tool HTTP callback, and translates
every tool call from the platform into a request against the campus
scene supervisor (port 8780). The agent does NOT chat with the per-
Husky bridges -- the scene supervisor owns all dispatch + state.

Run:

    export OMNI_KEY="olink_..."
    pip install omnilink
    python agents/templates/mission_control/mission_control_agent.py

Then open https://omnilink-agents.com, pick "MissionControl", and chat.
"""

from __future__ import annotations

import http.server
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace")  # type: ignore[attr-defined]
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


THIS_DIR = Path(__file__).resolve().parent
PROFILE_PATH = THIS_DIR / "profile.json"
BASE_URL = os.environ.get("OMNILINK_BASE_URL", "https://www.omnilink-agents.com")

MC_PORT = int(os.environ.get("MISSION_CONTROL_PORT", "51530"))
SCENE_URL = os.environ.get("MC_SCENE_URL", "http://127.0.0.1:8780").rstrip("/")
POLL_INTERVAL_S = float(os.environ.get("MISSION_CONTROL_POLL_S", "3"))


def _scene_get(path: str, timeout: float = 5.0) -> Dict[str, Any]:
    try:
        req = urllib.request.Request(f"{SCENE_URL}{path}", method="GET")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            detail = ""
        return {"error": f"scene HTTP {e.code}", "detail": detail[:300]}
    except Exception as e:
        return {"error": f"scene call failed: {e}"}


def _scene_post(path: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 10.0) -> Dict[str, Any]:
    body = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(f"{SCENE_URL}{path}", data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:
        try:
            detail = e.read().decode("utf-8", errors="replace")
        except Exception:
            detail = ""
        return {"error": f"scene HTTP {e.code}", "detail": detail[:300]}
    except Exception as e:
        return {"error": f"scene call failed: {e}"}


# ── Tool impls ─────────────────────────────────────────────────────


def tool_list_zones(**_: Any) -> Dict[str, Any]:
    return _scene_get("/list_zones")


def tool_get_zone(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name: return {"error": "name is required"}
    return _scene_get(f"/get_zone/{name}")


def tool_list_robots(**_: Any) -> Dict[str, Any]:
    return _scene_get("/list_robots")


def tool_get_robot(robot_id: str = "", **_: Any) -> Dict[str, Any]:
    if not robot_id: return {"error": "robot_id is required"}
    return _scene_get(f"/get_robot/{robot_id}")


def tool_distances_to(zone: str = "", **_: Any) -> Dict[str, Any]:
    if not zone: return {"error": "zone is required"}
    return _scene_get(f"/distances_to/{zone}")


def tool_dispatch(robot_id: str = "", zone: str = "", **_: Any) -> Dict[str, Any]:
    if not robot_id: return {"error": "robot_id is required"}
    if not zone: return {"error": "zone is required"}
    return _scene_post("/dispatch", {"robot_id": robot_id, "zone": zone})


def tool_cancel(robot_id: str = "", **_: Any) -> Dict[str, Any]:
    if not robot_id: return {"error": "robot_id is required"}
    return _scene_post("/cancel", {"robot_id": robot_id})


def tool_cancel_all(**_: Any) -> Dict[str, Any]:
    return _scene_post("/cancel_all", {})


def tool_reset_fleet(**_: Any) -> Dict[str, Any]:
    return _scene_post("/reset_fleet", {})


def tool_save_route(name: str = "", zones: Optional[List[str]] = None, **_: Any) -> Dict[str, Any]:
    if not name: return {"error": "name is required"}
    if not zones or not isinstance(zones, list):
        return {"error": "zones must be a non-empty list"}
    return _scene_post("/save_route", {"name": name, "zones": zones})


def tool_list_routes(**_: Any) -> Dict[str, Any]:
    return _scene_get("/list_routes")


def tool_run_route(robot_id: str = "", route: str = "", **_: Any) -> Dict[str, Any]:
    if not robot_id: return {"error": "robot_id is required"}
    if not route: return {"error": "route is required"}
    return _scene_post("/run_route", {"robot_id": robot_id, "route": route})


def tool_forget_route(name: str = "", **_: Any) -> Dict[str, Any]:
    if not name: return {"error": "name is required"}
    return _scene_post("/forget_route", {"name": name})


def tool_record_observation(robot_id: str = "", text: str = "",
                            zone: str = "", **_: Any) -> Dict[str, Any]:
    if not robot_id: return {"error": "robot_id is required"}
    if not text: return {"error": "text is required"}
    payload: Dict[str, Any] = {"robot_id": robot_id, "text": text}
    if zone:
        payload["zone"] = zone
    return _scene_post("/record_observation", payload)


def tool_list_observations(robot_id: str = "", zone: str = "", **_: Any) -> Dict[str, Any]:
    r = _scene_get("/list_observations")
    obs = r.get("observations", [])
    if robot_id:
        obs = [o for o in obs if o.get("robot_id") == robot_id]
    if zone:
        obs = [o for o in obs if o.get("zone") == zone]
    return {"observations": obs, "count": len(obs)}


def tool_get_activity_log(last_n: int = 30, **_: Any) -> Dict[str, Any]:
    r = _scene_get("/activity")
    entries = r.get("entries", [])
    return {"entries": entries[-int(last_n):], "count": min(len(entries), int(last_n))}


TOOLS: Dict[str, Dict[str, Any]] = {
    # Discovery
    "list_zones": {
        "description": "List all 12 named zones on the campus with coordinates, color, kind, and description. Call once at the start so you know the campus vocabulary.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_zones,
    },
    "get_zone": {
        "description": "Look up one zone's coordinates + description by name.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_get_zone,
    },
    "list_robots": {
        "description": "Snapshot of all 6 Huskies: id, position, status (busy/idle), current task, queue depth, observations recorded so far.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_robots,
    },
    "get_robot": {
        "description": "Detail for one Husky including its current queue. Use when planning a single-robot multi-step task.",
        "parameters": {"type": "object", "properties": {"robot_id": {"type": "string"}}, "required": ["robot_id"]},
        "impl": tool_get_robot,
    },
    "distances_to": {
        "description": "Distance from every robot to a named zone, sorted ascending. The right primitive for 'send the closest available' -- pick the first robot with status='idle'.",
        "parameters": {"type": "object", "properties": {"zone": {"type": "string"}}, "required": ["zone"]},
        "impl": tool_distances_to,
    },
    # Dispatch
    "dispatch": {
        "description": "Send one Husky to one named zone. Non-blocking. If the robot is busy, the dispatch is queued FIFO. Returns {accepted, queued, queue_position}.",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string"},
                "zone": {"type": "string"},
            }, "required": ["robot_id", "zone"]},
        "impl": tool_dispatch,
    },
    "cancel": {
        "description": "Abort one robot's current task + clear its dispatch queue. Use to interrupt before issuing a new priority dispatch.",
        "parameters": {"type": "object", "properties": {"robot_id": {"type": "string"}}, "required": ["robot_id"]},
        "impl": tool_cancel,
    },
    "cancel_all": {
        "description": "Abort every robot's current task. Use on 'stop everything' / 'forget all that' / new priorities.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_cancel_all,
    },
    "reset_fleet": {
        "description": "Recall every robot to its parking spot at the south edge. Use to end a mission cleanly.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_reset_fleet,
    },
    # Routes
    "save_route": {
        "description": "Save a named sequence of zones. Run later via run_route. Persistent only in the running scene's memory.",
        "parameters": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "zones": {"type": "array", "items": {"type": "string"}},
            }, "required": ["name", "zones"]},
        "impl": tool_save_route,
    },
    "list_routes": {
        "description": "All saved routes.",
        "parameters": {"type": "object", "properties": {}},
        "impl": tool_list_routes,
    },
    "run_route": {
        "description": "Run a saved route on one robot. The robot visits each zone in order (first via dispatch, rest queued).",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string"},
                "route": {"type": "string"},
            }, "required": ["robot_id", "route"]},
        "impl": tool_run_route,
    },
    "forget_route": {
        "description": "Delete a saved route.",
        "parameters": {"type": "object", "properties": {"name": {"type": "string"}}, "required": ["name"]},
        "impl": tool_forget_route,
    },
    # Observations
    "record_observation": {
        "description": "Record a free-form note about what a robot found at a zone. Use to surface mission findings the operator can later query.",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string"},
                "text": {"type": "string"},
                "zone": {"type": "string", "description": "Optional; defaults to the robot's current target zone."},
            }, "required": ["robot_id", "text"]},
        "impl": tool_record_observation,
    },
    "list_observations": {
        "description": "Read recorded observations. Optional robot_id / zone filter.",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string"},
                "zone": {"type": "string"},
            }},
        "impl": tool_list_observations,
    },
    # Observability
    "get_activity_log": {
        "description": "Recent scene activity: dispatches, arrivals, cancels, observations. Use to answer 'what happened just now' / 'who finished what'.",
        "parameters": {"type": "object", "properties": {"last_n": {"type": "integer"}}},
        "impl": tool_get_activity_log,
    },
}


def dispatch_tool(name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    spec = TOOLS.get(name)
    if spec is None:
        return {"error": f"unknown tool: {name}", "known": sorted(TOOLS.keys())}
    try:
        return spec["impl"](**args)
    except TypeError as e:
        return {"error": f"bad args: {e}"}
    except Exception as e:
        return {"error": f"tool crashed: {e.__class__.__name__}: {e}"}


# ── Tool callback HTTP server ───────────────────────────────────

ACTIVITY: List[Dict[str, Any]] = []


def make_handler():
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_): return

        def _cors(self):
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def do_OPTIONS(self):
            self.send_response(200); self._cors(); self.end_headers()

        def do_GET(self):
            if self.path == "/health":
                body = json.dumps({"ok": True, "tools": sorted(TOOLS.keys()),
                                   "scene_url": SCENE_URL}).encode()
            elif self.path == "/activity":
                body = json.dumps({"entries": ACTIVITY[-100:]}, default=str).encode()
            else:
                self.send_error(404); return
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self._cors(); self.end_headers(); self.wfile.write(body)

        def do_POST(self):
            if self.path != "/tool":
                self.send_error(404); return
            n = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(n).decode("utf-8", errors="replace")
            try:
                data = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                data = {}
            tool_name = data.pop("tool", "")
            print(f"  [TOOL] {tool_name}({data})")
            result = dispatch_tool(tool_name, data)
            ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            ACTIVITY.append({"ts": ts, "tool": tool_name, "args": data, "result": result})
            body = json.dumps({"status": "ok", "tool": tool_name, "result": result},
                              default=str).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json")
            self._cors(); self.end_headers(); self.wfile.write(body)

    return H


def start_tool_server() -> int:
    try:
        server = http.server.ThreadingHTTPServer(("0.0.0.0", MC_PORT), make_handler())
    except OSError as e:
        print(f"  [WARN] port {MC_PORT} taken ({e}); falling back to random.")
        server = http.server.ThreadingHTTPServer(("0.0.0.0", 0), make_handler())
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return port


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
        print("ERROR: OMNI_KEY not set.")
        return 1

    profile = load_profile()
    agent_name = profile["name"]
    base_settings = profile["settings"]

    port = start_tool_server()
    tool_callback = f"http://127.0.0.1:{port}/tool"
    print(f"  MissionControl tool server: {tool_callback}")
    print(f"  Scene supervisor URL:       {SCENE_URL}")
    print(f"  {len(TOOLS)} tools registered")

    # Probe the scene supervisor at startup.
    h = _scene_get("/health")
    if "error" in h:
        print(f"  WARNING: scene unreachable at startup -- {h['error']}")
    else:
        print(f"  Scene healthy: {h.get('robots')} robots, {h.get('zones')} zones")

    client = OmniLinkClient(omni_key=omni_key, base_url=BASE_URL, timeout=30)
    settings = build_settings(base_settings, tool_callback)
    ensure_profile(client, agent_name, settings)

    print(f"  Ready. Open https://omnilink-agents.com and pick {agent_name!r}.")
    print(f"  Ctrl-C to exit.")
    try:
        while True:
            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
