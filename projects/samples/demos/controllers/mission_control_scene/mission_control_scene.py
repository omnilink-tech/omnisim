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

"""Mission Control scene supervisor for omnilink_mission_control.wbt.

Owns:

  - the named-zone catalog (name -> {x, y, color, description, kind})
  - per-robot state (position, current task, idle/busy, last_seen_at)
  - a dispatch queue (which robot is going where, started_at, eta)
  - persistent named routes (custom sequences of zones)
  - operator observations (free-form notes the agent records about
    what it found at each zone)

HTTP surface on :8780. Thread-safe: every Supervisor call (getField,
getPosition, setSFVec3f) goes through the main controller tick via the
pending-op queue. HTTP handlers only read from / append to the
in-memory snapshot.

The agent does NOT call the Husky bridges directly. The scene
supervisor is the single point of contact -- it teleports the Huskies
to dispatch them, reads their poses each tick, and tracks task
completion. This makes the agent's tool surface clean (12 endpoints
total) and means the fleet's behaviour is consistent regardless of
how the operator phrases their mission.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="backslashreplace")  # type: ignore[attr-defined]
    except Exception:
        pass

from omnisim import Supervisor

HTTP_PORT = int(os.environ.get("MC_BRIDGE_PORT", "8780"))
# How close (in metres) the Husky has to be to consider a dispatch
# complete. With supervisor teleport we hit this in one tick; for a
# real-driving deployment 0.5 m is forgiving.
DISPATCH_ARRIVE_M = float(os.environ.get("MC_DISPATCH_ARRIVE_M", "0.5"))


# Twelve zones. (name, x, y, color, kind, description).
ZONES: List[Tuple[str, float, float, str, str, str]] = [
    ("north_gate",       0.0,  12.0, "yellow", "gate",     "Main entrance, north side"),
    ("south_gate",       0.0, -12.0, "yellow", "gate",     "Service entrance, south side"),
    ("east_gate",       12.0,   0.0, "yellow", "gate",     "Vehicle access, east"),
    ("west_gate",      -12.0,   0.0, "yellow", "gate",     "Vehicle access, west"),
    ("dock_a",          -8.0,   8.0, "blue",   "dock",     "Loading dock A (NW)"),
    ("dock_b",           8.0,   8.0, "blue",   "dock",     "Loading dock B (NE)"),
    ("dock_c",          -8.0,  -8.0, "blue",   "dock",     "Loading dock C (SW)"),
    ("dock_d",           8.0,  -8.0, "blue",   "dock",     "Loading dock D (SE)"),
    ("warehouse",        0.0,   5.0, "gray",   "storage",  "Central storage warehouse"),
    ("lab",             -5.0,   0.0, "green",  "facility", "Research lab"),
    ("cafeteria",        5.0,   0.0, "orange", "facility", "Workers' rest area"),
    ("charging_station", 0.0,   0.0, "white",  "service",  "Robot charging hub at campus centre"),
]

# Six Huskies, parked at the south edge. (id, parking_x, parking_y, yaw_deg).
HUSKIES: List[Tuple[str, float, float, float]] = [
    ("husky_1", -5.0, -13.0, 90.0),
    ("husky_2", -3.0, -13.0, 90.0),
    ("husky_3", -1.0, -13.0, 90.0),
    ("husky_4",  1.0, -13.0, 90.0),
    ("husky_5",  3.0, -13.0, 90.0),
    ("husky_6",  5.0, -13.0, 90.0),
]


def _now_iso() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ─── Scene state ──────────────────────────────────────────────────

class HuskyState:
    def __init__(self, hid: str, parking: Tuple[float, float], yaw_deg: float):
        self.id = hid
        self.parking = parking
        self.parking_yaw = yaw_deg
        self.node: Any = None
        # Live state
        self.x = parking[0]
        self.y = parking[1]
        self.task: Optional[str] = None       # human-readable current task
        self.target_zone: Optional[str] = None
        self.target_xy: Optional[Tuple[float, float]] = None
        self.dispatched_at: float = 0.0
        self.completed_at: float = 0.0
        self.observation_count: int = 0
        # Pending: a queue of {kind, xy, zone_name} for run_route.
        self.queue: List[Dict[str, Any]] = []


class Scene:
    def __init__(self, robot: Supervisor):
        self.robot = robot
        self.lock = threading.RLock()
        self.timestep = int(robot.getBasicTimeStep())

        self.zones: Dict[str, Dict[str, Any]] = {}
        for name, x, y, color, kind, desc in ZONES:
            self.zones[name] = {
                "name": name, "x": x, "y": y,
                "color": color, "kind": kind, "description": desc,
            }

        self.huskies: Dict[str, HuskyState] = {}
        for hid, px, py, yaw in HUSKIES:
            hs = HuskyState(hid, (px, py), yaw)
            hs.node = robot.getFromDef(hid.upper())
            if hs.node is None:
                print(f"[mc_scene] WARNING: DEF {hid.upper()} not found")
            self.huskies[hid] = hs

        self.routes: Dict[str, List[str]] = {}
        self.observations: List[Dict[str, Any]] = []
        self.activity_log: List[Dict[str, Any]] = []
        self.pending_ops: List[Tuple[Any, threading.Event, Dict[str, Any]]] = []

    # ── Helpers ────────────────────────────────────────────────

    def _log(self, kind: str, **info: Any) -> None:
        entry = {"ts": _now_iso(), "kind": kind}
        entry.update(info)
        self.activity_log.append(entry)
        if len(self.activity_log) > 500:
            del self.activity_log[:100]

    def _distance(self, ax: float, ay: float, bx: float, by: float) -> float:
        return math.hypot(ax - bx, ay - by)

    # ── Public read-only ───────────────────────────────────────

    def list_zones(self) -> List[Dict[str, Any]]:
        return [dict(z) for z in self.zones.values()]

    def get_zone(self, name: str) -> Optional[Dict[str, Any]]:
        z = self.zones.get(name)
        return dict(z) if z else None

    def list_robots(self) -> List[Dict[str, Any]]:
        out = []
        with self.lock:
            for h in self.huskies.values():
                out.append({
                    "id": h.id,
                    "position": [round(h.x, 2), round(h.y, 2)],
                    "task": h.task,
                    "target_zone": h.target_zone,
                    "status": "busy" if h.task else "idle",
                    "queue_depth": len(h.queue),
                    "observation_count": h.observation_count,
                })
        return out

    def get_robot(self, hid: str) -> Optional[Dict[str, Any]]:
        with self.lock:
            h = self.huskies.get(hid)
            if h is None: return None
            return {
                "id": h.id,
                "position": [round(h.x, 2), round(h.y, 2)],
                "task": h.task,
                "target_zone": h.target_zone,
                "status": "busy" if h.task else "idle",
                "queue_depth": len(h.queue),
                "queue": list(h.queue),
                "observation_count": h.observation_count,
                "dispatched_at": h.dispatched_at,
                "completed_at": h.completed_at,
            }

    def distances_to(self, zone_name: str) -> Dict[str, Any]:
        z = self.zones.get(zone_name)
        if z is None:
            return {"error": f"unknown zone {zone_name!r}",
                    "known": sorted(self.zones.keys())}
        out = []
        with self.lock:
            for h in self.huskies.values():
                d = self._distance(h.x, h.y, z["x"], z["y"])
                out.append({
                    "id": h.id, "distance_m": round(d, 2),
                    "status": "busy" if h.task else "idle",
                })
        out.sort(key=lambda r: r["distance_m"])
        return {"zone": zone_name, "robots": out}

    def list_routes(self) -> List[Dict[str, Any]]:
        return [{"name": k, "zones": list(v), "length": len(v)}
                for k, v in self.routes.items()]

    def get_route(self, name: str) -> Optional[Dict[str, Any]]:
        v = self.routes.get(name)
        if v is None: return None
        return {"name": name, "zones": list(v), "length": len(v)}

    def list_observations(self, robot_id: Optional[str] = None,
                          zone: Optional[str] = None) -> List[Dict[str, Any]]:
        with self.lock:
            out = list(self.observations)
        if robot_id:
            out = [o for o in out if o.get("robot_id") == robot_id]
        if zone:
            out = [o for o in out if o.get("zone") == zone]
        return out

    def get_activity_log(self, last_n: int = 50) -> List[Dict[str, Any]]:
        return list(self.activity_log[-int(last_n):])

    # ── Public mutating (queue + state) ────────────────────────

    def dispatch(self, hid: str, zone_name: str) -> Dict[str, Any]:
        """Send a robot to a zone. If busy, the dispatch is queued."""
        with self.lock:
            h = self.huskies.get(hid)
            if h is None:
                return {"error": f"unknown robot {hid!r}",
                        "known": sorted(self.huskies.keys())}
            z = self.zones.get(zone_name)
            if z is None:
                return {"error": f"unknown zone {zone_name!r}",
                        "known": sorted(self.zones.keys())}
            queued = {"kind": "dispatch", "zone": zone_name,
                      "xy": (z["x"], z["y"]),
                      "task": f"go to {zone_name}"}
            if h.task is not None:
                h.queue.append(queued)
                self._log("dispatch_queued", robot=hid, zone=zone_name,
                          queue_depth=len(h.queue))
                return {"accepted": True, "queued": True,
                        "robot_id": hid, "zone": zone_name,
                        "queue_position": len(h.queue)}
            # Idle: start immediately. The actual teleport is queued for
            # the main tick.
            h.task = f"go to {zone_name}"
            h.target_zone = zone_name
            h.target_xy = (z["x"], z["y"])
            h.dispatched_at = time.time()
            self._enqueue_op({"kind": "teleport",
                              "hid": hid, "xy": (z["x"], z["y"]),
                              "yaw_deg": 0.0})
            self._log("dispatch", robot=hid, zone=zone_name)
            return {"accepted": True, "queued": False,
                    "robot_id": hid, "zone": zone_name}

    def cancel_robot(self, hid: str) -> Dict[str, Any]:
        with self.lock:
            h = self.huskies.get(hid)
            if h is None:
                return {"error": f"unknown robot {hid!r}"}
            prev_task = h.task
            prev_queue = list(h.queue)
            h.task = None
            h.target_zone = None
            h.target_xy = None
            h.queue.clear()
            self._log("cancel", robot=hid, prev_task=prev_task,
                      cancelled_queue=len(prev_queue))
            return {"cancelled": True, "robot_id": hid,
                    "prev_task": prev_task, "cleared_queue": len(prev_queue)}

    def cancel_all(self) -> Dict[str, Any]:
        out = {}
        with self.lock:
            for hid in list(self.huskies):
                out[hid] = self.cancel_robot(hid)
        return {"cancelled_all": True, "details": out}

    def reset_fleet(self) -> Dict[str, Any]:
        """Return every Husky to its parking pose and clear all tasks."""
        with self.lock:
            for h in self.huskies.values():
                h.task = None
                h.target_zone = None
                h.target_xy = None
                h.queue.clear()
                self._enqueue_op({"kind": "teleport", "hid": h.id,
                                  "xy": h.parking, "yaw_deg": h.parking_yaw})
            self._log("reset_fleet")
        return {"reset": True, "count": len(self.huskies)}

    def save_route(self, name: str, zones: List[str]) -> Dict[str, Any]:
        unknown = [z for z in zones if z not in self.zones]
        if unknown:
            return {"error": "unknown zones", "unknown": unknown,
                    "known": sorted(self.zones.keys())}
        with self.lock:
            self.routes[name] = list(zones)
        self._log("save_route", name=name, length=len(zones))
        return {"saved": True, "name": name, "length": len(zones)}

    def forget_route(self, name: str) -> Dict[str, Any]:
        with self.lock:
            removed = self.routes.pop(name, None)
        return {"forgotten": removed is not None, "name": name}

    def run_route(self, hid: str, route_name: str) -> Dict[str, Any]:
        with self.lock:
            h = self.huskies.get(hid)
            if h is None:
                return {"error": f"unknown robot {hid!r}"}
            route = self.routes.get(route_name)
            if route is None:
                return {"error": f"unknown route {route_name!r}",
                        "known": sorted(self.routes.keys())}
            if not route:
                return {"error": "route is empty"}
        # First zone via dispatch, rest queued.
        first = route[0]
        res = self.dispatch(hid, first)
        if "error" in res:
            return res
        with self.lock:
            for z in route[1:]:
                zinfo = self.zones[z]
                self.huskies[hid].queue.append(
                    {"kind": "dispatch", "zone": z,
                     "xy": (zinfo["x"], zinfo["y"]),
                     "task": f"go to {z}"}
                )
        self._log("run_route", robot=hid, route=route_name, length=len(route))
        return {"accepted": True, "robot_id": hid, "route": route_name,
                "queued_legs": len(route) - 1}

    def record_observation(self, robot_id: str, text: str,
                           zone: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            h = self.huskies.get(robot_id)
            if h is None:
                return {"error": f"unknown robot {robot_id!r}"}
            entry = {
                "ts": _now_iso(),
                "robot_id": robot_id,
                "zone": zone or h.target_zone,
                "position": [round(h.x, 2), round(h.y, 2)],
                "text": text,
            }
            self.observations.append(entry)
            h.observation_count += 1
            self._log("observation", robot=robot_id, zone=entry["zone"])
        return {"recorded": True, "entry": entry}

    # ── Op queue + main tick ───────────────────────────────────

    def _enqueue_op(self, op: Dict[str, Any]) -> None:
        evt = threading.Event()
        result: Dict[str, Any] = {}
        with self.lock:
            self.pending_ops.append((op, evt, result))

    def tick(self) -> None:
        """Main controller thread. Drains pending Supervisor ops, updates
        the live pose snapshot, and advances per-robot task state."""
        # 1. Drain pending Supervisor ops (teleports).
        with self.lock:
            ops = list(self.pending_ops)
            self.pending_ops.clear()
        for op, evt, result in ops:
            try:
                kind = op.get("kind")
                if kind == "teleport":
                    hid = op["hid"]
                    h = self.huskies.get(hid)
                    if h is not None and h.node is not None:
                        x, y = op["xy"]
                        yaw = op.get("yaw_deg", 0.0)
                        try:
                            tr = h.node.getField("translation")
                            if tr is not None:
                                tr.setSFVec3f([float(x), float(y), 0.2])
                            rot = h.node.getField("rotation")
                            if rot is not None:
                                rot.setSFRotation([0.0, 0.0, 1.0,
                                                   math.radians(yaw)])
                            try:
                                h.node.resetPhysics()
                            except Exception:
                                pass
                        except Exception as e:
                            print(f"[mc_scene] teleport {hid} failed: {e}")
                result.update({"ok": True})
            except Exception as e:
                result.update({"error": f"op crashed: {e}"})
            evt.set()

        # 2. Update live pose snapshot from each Husky node.
        with self.lock:
            for h in self.huskies.values():
                if h.node is None: continue
                try:
                    p = h.node.getPosition()
                    if p is not None and len(p) >= 2:
                        h.x = float(p[0])
                        h.y = float(p[1])
                except Exception:
                    pass

        # 3. Advance task state: mark dispatch complete when robot is
        # within arrival radius of its target. Pop the next queued
        # leg if any.
        with self.lock:
            for h in self.huskies.values():
                if h.task is None or h.target_xy is None:
                    continue
                d = self._distance(h.x, h.y, h.target_xy[0], h.target_xy[1])
                if d < DISPATCH_ARRIVE_M:
                    h.completed_at = time.time()
                    self._log("arrived", robot=h.id, zone=h.target_zone,
                              elapsed_s=round(h.completed_at - h.dispatched_at, 2))
                    h.task = None
                    h.target_zone = None
                    h.target_xy = None
                    # If the queue has more legs, start the next.
                    if h.queue:
                        next_leg = h.queue.pop(0)
                        h.task = next_leg.get("task", "next leg")
                        h.target_zone = next_leg.get("zone")
                        h.target_xy = next_leg.get("xy")
                        h.dispatched_at = time.time()
                        if h.target_xy is not None:
                            self._enqueue_op({
                                "kind": "teleport", "hid": h.id,
                                "xy": h.target_xy, "yaw_deg": 0.0,
                            })


# ─── HTTP layer ────────────────────────────────────────────────────

def make_handler(scene: Scene):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *_: Any) -> None: return

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
            if n <= 0: return {}
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
            p = self.path
            if p == "/health":
                return self._json(200, {"ok": True,
                                         "robots": len(scene.huskies),
                                         "zones": len(scene.zones)})
            if p == "/list_zones":
                return self._json(200, {"zones": scene.list_zones()})
            if p.startswith("/get_zone/"):
                name = p[len("/get_zone/"):]
                z = scene.get_zone(name)
                return self._json(200 if z else 404,
                                  z or {"error": f"unknown zone {name!r}"})
            if p == "/list_robots":
                return self._json(200, {"robots": scene.list_robots()})
            if p.startswith("/get_robot/"):
                rid = p[len("/get_robot/"):]
                r = scene.get_robot(rid)
                return self._json(200 if r else 404,
                                  r or {"error": f"unknown robot {rid!r}"})
            if p.startswith("/distances_to/"):
                name = p[len("/distances_to/"):]
                return self._json(200, scene.distances_to(name))
            if p == "/list_routes":
                return self._json(200, {"routes": scene.list_routes()})
            if p.startswith("/get_route/"):
                name = p[len("/get_route/"):]
                r = scene.get_route(name)
                return self._json(200 if r else 404,
                                  r or {"error": f"unknown route {name!r}"})
            if p == "/list_observations":
                return self._json(200, {"observations": scene.list_observations()})
            if p == "/activity":
                return self._json(200, {"entries": scene.get_activity_log()})
            return self._json(404, {"error": "not_found"})

        def do_POST(self):
            body = self._read_json()
            p = self.path
            if p == "/dispatch":
                return self._json(200, scene.dispatch(
                    body.get("robot_id", ""),
                    body.get("zone", "")))
            if p == "/cancel":
                return self._json(200, scene.cancel_robot(body.get("robot_id", "")))
            if p == "/cancel_all":
                return self._json(200, scene.cancel_all())
            if p == "/reset_fleet":
                return self._json(200, scene.reset_fleet())
            if p == "/save_route":
                return self._json(200, scene.save_route(
                    body.get("name", ""),
                    body.get("zones", [])))
            if p == "/forget_route":
                return self._json(200, scene.forget_route(body.get("name", "")))
            if p == "/run_route":
                return self._json(200, scene.run_route(
                    body.get("robot_id", ""),
                    body.get("route", "")))
            if p == "/record_observation":
                return self._json(200, scene.record_observation(
                    body.get("robot_id", ""),
                    body.get("text", ""),
                    body.get("zone")))
            return self._json(404, {"error": "not_found"})

    return H


def start_http(scene: Scene) -> None:
    server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), make_handler(scene))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[mc_scene] HTTP on http://127.0.0.1:{HTTP_PORT}")


def main() -> int:
    robot = Supervisor()
    scene = Scene(robot)
    start_http(scene)
    print(f"[mc_scene] {len(scene.huskies)} huskies, "
          f"{len(scene.zones)} zones")
    timestep = scene.timestep
    while robot.step(timestep) != -1:
        scene.tick()
    return 0


if __name__ == "__main__":
    sys.exit(main())
