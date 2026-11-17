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

"""CourierBridge — the OMNITUG500 warehouse-courier action surface + executor.

Owns the rover (the physics scanner-sidecar body), the visual OMNITUG500 it drags
along, the staged packages, and the named stations. Exposes high-level courier
actions — goto a named station, pick a package, deliver to a dock, or run a
whole multi-stop route — that an operator or the OmniLink agent drives in
natural language.

Threading contract (same as omnilink_mobile_bridge): HTTP / chat handlers only
MUTATE plain Python state under `self.lock` (enqueue steps, set flags). EVERY
Webots call happens in `tick()`, which the controller's main loop invokes once
per physics step. `get_state()` returns values cached by the last tick, so a
status poll never touches the simulator from another thread.

Pick/deliver model: the OMNITUG500 is an AGV with no arm, so a "pick" is a deck
load — when the rover is parked alongside a bay, the staged package is placed
onto a deck slot and rides there (pose-forced + velocity-zeroed each tick, the
warm-solver carry recipe). "deliver" sets the carried package(s) down at the
dock drop point, where their own physics settles them.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

from courier_nav import CourierNav

# ── drive / control constants ─────────────────────────────────────────
V_MAX = 0.70          # m/s cruise
W_MAX = 1.60          # rad/s
LOOKAHEAD = 0.70      # pure-pursuit carrot distance (m); tighter = less corner-cut
STEER_KP = 1.20
DW_MAX = 0.30         # max change in yaw-rate per tick (no jerk)
APPROACH_KV = 1.2     # slow down near the goal (v <= APPROACH_KV * dist)
GOAL_TOL = 0.28       # arrival radius at an anchor (m)
TURN_KP = 2.5
TURN_W = 1.30
ALIGN_TOL = 0.05      # heading alignment tolerance (rad)
R_GUARD = 0.60        # static-obstacle safety guard radius (m)
STEP_TIMEOUT_S = 150.0  # per-step watchdog, in SIM seconds (robot.getTime)


def clampf(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class CourierBridge:
    def __init__(self, robot, layout: dict, ts_ms: int) -> None:
        self.robot = robot
        self.layout = layout
        self.ts_ms = ts_ms
        self.dt = ts_ms / 1000.0
        self.nav = CourierNav(layout)

        self.me = robot.getSelf()                 # SCANNERS physics body
        self.visual = robot.getFromDef("OMNITUG500")  # visual rover (teleported to match)
        self._mtf = self.me.getField("translation")
        self._mrf = self.me.getField("rotation")
        self._vtf = self.visual.getField("translation") if self.visual else None
        self._vrf = self.visual.getField("rotation") if self.visual else None

        self.stations: Dict[str, dict] = {s["name"]: s for s in layout["stations"]}
        self.deck_slots: List[List[float]] = layout["deck_slots"]
        sp = layout["spawn"]
        self.spawn_xy = (sp["x"], sp["y"])
        self.spawn_yaw = sp["heading"] - math.pi / 2.0

        # package nodes + their original staging poses (for reset)
        self.packages: Dict[str, dict] = {}
        for pk in layout["packages"]:
            node = robot.getFromDef(pk["def"])
            self.packages[pk["name"]] = {
                "name": pk["name"], "node": node, "station": pk["station"],
                "spawn": list(pk["spawn"]), "color": pk.get("color"),
            }

        # ── live state (cached for get_state) ──
        self.lock = threading.RLock()
        self.x = self.spawn_xy[0]
        self.y = self.spawn_xy[1]
        self.yaw = self.spawn_yaw          # rover yaw_z (rotation about +Z)
        self.speed = 0.0
        self.carry: List[dict] = []        # [{name, slot}]
        self.delivered_at: Dict[str, int] = {}   # dock -> count (for stacking)
        self.queue: deque = deque()        # pending steps
        self.active: Optional[dict] = None
        self.phase = "idle"                # idle | drive | align | act
        self.path: List[Tuple[float, float]] = []
        self._path_i = 0                   # pure-pursuit progress index (only advances)
        self.last_event = "ready"
        self.fault: Optional[str] = None
        self._stop = False
        self._reset = False
        self._prev_omega = 0.0
        self._active_since = 0.0
        self.window_outbox: List[str] = []
        self.window_configured = False

    # ── window queue ──────────────────────────────────────────────
    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    # ── station helpers ───────────────────────────────────────────
    def _station(self, name: Optional[str]) -> Optional[dict]:
        if not name:
            return None
        return self.stations.get(name)

    def station_names(self, kind: Optional[str] = None) -> List[str]:
        return [n for n, s in self.stations.items()
                if kind is None or s["kind"] == kind]

    # ── thread-safe action surface (enqueue only) ─────────────────
    def act_goto(self, station: str) -> dict:
        st = self._station(station)
        if st is None:
            return {"error": f"unknown station '{station}'",
                    "known": self.station_names()}
        with self.lock:
            self.queue.append({"op": "goto", "station": station})
        return {"accepted": True, "op": "goto", "station": station,
                "eta_s": self._eta(st)}

    def act_pick(self, station: Optional[str] = None,
                 package: Optional[str] = None) -> dict:
        # Resolve station: explicit, else the bay where `package` is staged.
        if station is None and package:
            pk = self.packages.get(package)
            if pk:
                station = pk["station"]
        st = self._station(station)
        if st is None or st["kind"] != "pickup":
            return {"error": f"'{station}' is not a pickup bay",
                    "bays": self.station_names("pickup")}
        if package and package not in self.packages:
            return {"error": f"unknown package '{package}'",
                    "packages": list(self.packages.keys())}
        with self.lock:
            self.queue.append({"op": "pick", "station": station, "package": package})
        return {"accepted": True, "op": "pick", "station": station,
                "package": package, "eta_s": self._eta(st)}

    def act_deliver(self, station: str, package: Optional[str] = None) -> dict:
        st = self._station(station)
        if st is None or st["kind"] not in ("dropoff", "home"):
            return {"error": f"'{station}' is not a dropoff dock",
                    "docks": self.station_names("dropoff")}
        with self.lock:
            self.queue.append({"op": "deliver", "station": station, "package": package})
        return {"accepted": True, "op": "deliver", "station": station,
                "package": package, "eta_s": self._eta(st)}

    def act_run_route(self, steps: List[dict]) -> dict:
        """Enqueue a multi-stop route. Each step is
        {"action": "goto"|"pick"|"deliver", "station": <name>, "package"?: <name>}.
        Validates the whole route up front so a typo fails fast."""
        norm: List[dict] = []
        for i, s in enumerate(steps or []):
            action = (s.get("action") or s.get("op") or "").strip().lower()
            station = s.get("station")
            package = s.get("package")
            if action not in ("goto", "pick", "deliver"):
                return {"error": f"step {i}: bad action '{action}'"}
            st = self._station(station)
            if st is None:
                return {"error": f"step {i}: unknown station '{station}'",
                        "known": self.station_names()}
            if action == "pick" and st["kind"] != "pickup":
                return {"error": f"step {i}: '{station}' is not a pickup bay"}
            if action == "deliver" and st["kind"] not in ("dropoff", "home"):
                return {"error": f"step {i}: '{station}' is not a dropoff dock"}
            norm.append({"op": action, "station": station, "package": package})
        if not norm:
            return {"error": "empty route"}
        with self.lock:
            self.queue.extend(norm)
        return {"accepted": True, "steps": len(norm),
                "route": [f"{s['op']} {s['station']}" for s in norm]}

    def act_stop(self) -> dict:
        with self.lock:
            self._stop = True
            self.queue.clear()
        return {"accepted": True, "halted_at": time.time()}

    def act_reset(self) -> dict:
        with self.lock:
            self._reset = True
            self.queue.clear()
        return {"accepted": True}

    def _eta(self, st: dict) -> float:
        ax, ay = st["anchor"]
        return round(math.hypot(ax - self.x, ay - self.y) / V_MAX + 2.0, 1)

    # ── state readout (cached; no Webots calls) ───────────────────
    def get_state(self) -> dict:
        with self.lock:
            return {
                "x": round(self.x, 3), "y": round(self.y, 3),
                "yaw_deg": round(math.degrees(self.yaw), 1),
                "heading_deg": round(math.degrees(self.yaw + math.pi / 2.0), 1),
                "speed": round(self.speed, 3),
                "mode": self.phase,
                "active": (f"{self.active['op']} {self.active['station']}"
                           if self.active else None),
                "queue": len(self.queue),
                "carrying": [c["name"] for c in self.carry],
                "deck_free": max(0, len(self.deck_slots) - len(self.carry)),
                "at_station": self._at_station(),
                "last_event": self.last_event,
                "fault": self.fault,
                "sim_time": round(self.robot.getTime(), 2),
            }

    def capabilities(self) -> dict:
        return {
            "model": "OmniTug 500",
            "class": "warehouse courier (AGV)",
            "deck_capacity": len(self.deck_slots),
            "max_linear_m_s": V_MAX,
            "max_angular_rad_s": W_MAX,
            "pickup_bays": [{"name": s["name"], "label": s.get("label"),
                             "color": s.get("color")}
                            for s in self.stations.values() if s["kind"] == "pickup"],
            "docks": [{"name": s["name"], "label": s.get("label")}
                      for s in self.stations.values() if s["kind"] == "dropoff"],
            "packages": [{"name": p["name"], "staged_at": p["station"]}
                         for p in self.packages.values()],
        }

    def _at_station(self) -> Optional[str]:
        if self.phase != "idle":
            return None
        best, bd = None, 0.5
        for n, s in self.stations.items():
            ax, ay = s["anchor"]
            d = math.hypot(ax - self.x, ay - self.y)
            if d < bd:
                best, bd = n, d
        return best

    # ── per-tick executor (main thread only) ──────────────────────
    def tick(self) -> None:
        # 1. read true pose, cache it, drag the visual rover along
        p = self.me.getPosition()
        o = self.me.getOrientation()
        self.x, self.y, self.yaw = p[0], p[1], math.atan2(o[3], o[0])
        try:
            v = self.me.getVelocity()
            self.speed = math.hypot(v[0], v[1])
        except Exception:
            self.speed = 0.0
        if self._vtf:
            self._vtf.setSFVec3f([self.x, self.y, 0.0])
            self._vrf.setSFRotation([0.0, 0.0, 1.0, self.yaw])

        # 2. carry: hold each loaded package on its deck slot
        self._update_carry()

        # 3. one-shot stop / reset requests
        if self._reset:
            self._do_reset()
            return
        if self._stop:
            self._stop = False
            self.active = None
            self.phase = "idle"
            self.path = []
            self.me.setVelocity([0.0] * 6)
            self.last_event = "stopped"
            return

        # 4. pull the next step
        if self.active is None:
            with self.lock:
                step = self.queue.popleft() if self.queue else None
            if step is None:
                self.me.setVelocity([0.0] * 6)
                self.phase = "idle"
                return
            self._begin_step(step)

        # 5. run the active step's phase machine
        if self.robot.getTime() - self._active_since > STEP_TIMEOUT_S:
            self.last_event = f"timeout on {self.active['op']} {self.active['station']}"
            self.fault = self.last_event
            self.active = None
            self.me.setVelocity([0.0] * 6)
            return
        if self.phase == "drive":
            self._drive_tick()
        elif self.phase == "align":
            self._align_tick()
        elif self.phase == "act":
            self._act()

    # ── step lifecycle ────────────────────────────────────────────
    def _begin_step(self, step: dict) -> None:
        self.active = step
        self._active_since = self.robot.getTime()
        st = self.stations[step["station"]]
        self.path = self.nav.plan((self.x, self.y), tuple(st["anchor"])) or []
        self._path_i = 0
        if not self.path:
            self.last_event = f"no route to {step['station']}"
            self.fault = self.last_event
            self.active = None
            self.phase = "idle"
            return
        self.fault = None
        self.phase = "drive"
        self.last_event = f"driving to {st.get('label', step['station'])}"
        self._prev_omega = 0.0

    def _drive_tick(self) -> None:
        st = self.stations[self.active["station"]]
        gx, gy = st["anchor"]
        dist = math.hypot(gx - self.x, gy - self.y)
        if dist < GOAL_TOL:
            self.me.setVelocity([0.0] * 6)
            self.phase = "align"
            return
        # pure-pursuit carrot — advance a monotonic progress index so the
        # carrot is always AHEAD of the rover (the path is planned once, so a
        # naive "first point >= lookahead" picks a now-behind point and stalls).
        while (self._path_i < len(self.path) - 1 and
               math.hypot(self.path[self._path_i][0] - self.x,
                          self.path[self._path_i][1] - self.y) >
               math.hypot(self.path[self._path_i + 1][0] - self.x,
                          self.path[self._path_i + 1][1] - self.y)):
            self._path_i += 1
        carrot = self.path[-1]
        for k in range(self._path_i, len(self.path)):
            if math.hypot(self.path[k][0] - self.x, self.path[k][1] - self.y) >= LOOKAHEAD:
                carrot = self.path[k]
                break
        desired_yaw = math.atan2(carrot[1] - self.y, carrot[0] - self.x) - math.pi / 2.0
        err = wrap(desired_yaw - self.yaw)
        omega = clampf(STEER_KP * err, -W_MAX, W_MAX)
        omega = self._prev_omega + clampf(omega - self._prev_omega, -DW_MAX, DW_MAX)
        self._prev_omega = omega
        v = V_MAX * max(0.0, 1.0 - abs(err) / 1.6)
        v = min(v, APPROACH_KV * dist)
        if abs(err) > 1.4:
            v = 0.0
        # static safety guard (should never fire with a correct plan)
        fwd = self.yaw + math.pi / 2.0
        nx = self.x + v * math.cos(fwd) * self.dt
        ny = self.y + v * math.sin(fwd) * self.dt
        if v > 0.0 and self.nav.blocked_within(nx, ny, R_GUARD):
            v = 0.0
        self.me.setVelocity([v * math.cos(fwd), v * math.sin(fwd), 0.0, 0.0, 0.0, omega])

    def _align_tick(self) -> None:
        st = self.stations[self.active["station"]]
        desired_yaw = st["heading"] - math.pi / 2.0
        err = wrap(desired_yaw - self.yaw)
        if abs(err) < ALIGN_TOL:
            self.me.setVelocity([0.0] * 6)
            self.phase = "act"
            return
        w = clampf(TURN_KP * err, -TURN_W, TURN_W)
        if 0 < abs(w) < 0.35:
            w = 0.35 * (1 if w > 0 else -1)
        self.me.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, w])

    def _act(self) -> None:
        op = self.active["op"]
        st = self.stations[self.active["station"]]
        self.me.setVelocity([0.0] * 6)
        if op == "goto":
            self.last_event = f"arrived at {st.get('label', st['name'])}"
        elif op == "pick":
            self._do_pick(st, self.active.get("package"))
        elif op == "deliver":
            self._do_deliver(st, self.active.get("package"))
        self.active = None
        self.phase = "idle"

    # ── pick / deliver ────────────────────────────────────────────
    def _do_pick(self, st: dict, package: Optional[str]) -> None:
        if len(self.carry) >= len(self.deck_slots):
            self.last_event = f"deck full ({len(self.carry)}); cannot pick at {st['name']}"
            return
        carried = {c["name"] for c in self.carry}
        delivered = set(self._delivered_names())
        cand = None
        if package:
            if package in carried:
                self.last_event = f"{package} already on the deck"
                return
            cand = self.packages.get(package)
        else:
            # the package staged at this bay (nearest, not already carried)
            best = None
            for pk in self.packages.values():
                if pk["station"] != st["name"] or pk["name"] in carried \
                        or pk["name"] in delivered or pk["node"] is None:
                    continue
                px = pk["node"].getPosition()
                d = math.hypot(px[0] - self.x, px[1] - self.y)
                if best is None or d < best[0]:
                    best = (d, pk)
            cand = best[1] if best else None
        if cand is None or cand["node"] is None:
            self.last_event = f"nothing to pick at {st.get('label', st['name'])}"
            return
        slot = len(self.carry)
        self.carry.append({"name": cand["name"], "slot": slot})
        try:
            cand["node"].resetPhysics()
        except Exception:
            pass
        self._place_on_deck(cand["node"], slot)
        self.last_event = f"loaded {cand['name']} ({len(self.carry)}/{len(self.deck_slots)} on deck)"
        self.queue_window(f"tool:pick:ok:{cand['name']} loaded")

    def _do_deliver(self, st: dict, package: Optional[str]) -> None:
        if not self.carry:
            self.last_event = f"nothing on the deck to deliver at {st.get('label', st['name'])}"
            return
        drop = st.get("drop_point") or st.get("anchor")
        to_drop = [c for c in self.carry if (package is None or c["name"] == package)]
        if not to_drop:
            self.last_event = f"{package} is not on the deck"
            return
        names = []
        for c in to_drop:
            pk = self.packages.get(c["name"])
            n = self.delivered_at.get(st["name"], 0)
            # set down in a short row in front of the dock
            dy = (n % 3 - 1) * 0.45
            dx = (n // 3) * 0.45
            node = pk["node"]
            if node is not None:
                node.getField("translation").setSFVec3f(
                    [drop[0] + dx, drop[1] + dy, pk["spawn"][2]])
                node.getField("rotation").setSFRotation([0, 0, 1, 0])
                try:
                    node.resetPhysics()
                    node.setVelocity([0.0] * 6)
                except Exception:
                    pass
            self.delivered_at[st["name"]] = n + 1
            names.append(c["name"])
        self.carry = [c for c in self.carry if c not in to_drop]
        # re-pack remaining packages onto the low deck slots
        for i, c in enumerate(self.carry):
            c["slot"] = i
        self.last_event = (f"delivered {', '.join(names)} to "
                           f"{st.get('label', st['name'])}")
        self.queue_window(f"tool:deliver:ok:{', '.join(names)} -> {st['name']}")

    def _delivered_names(self) -> List[str]:
        """Packages currently sitting at a dock drop point (not on the deck,
        not at their original bay)."""
        out = []
        carried = {c["name"] for c in self.carry}
        for st in self.stations.values():
            if st["kind"] not in ("dropoff", "home"):
                continue
            dp = st.get("drop_point") or st.get("anchor")
            for pk in self.packages.values():
                if pk["name"] in carried or pk["node"] is None:
                    continue
                pp = pk["node"].getPosition()
                if math.hypot(pp[0] - dp[0], pp[1] - dp[1]) < 1.5:
                    out.append(pk["name"])
        return out

    # ── carry geometry ────────────────────────────────────────────
    def _deck_world(self, slot: int):
        """World pose of a deck slot, given the rover's current pose."""
        sx, sy, sz = self.deck_slots[slot]
        cy, syaw = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + cy * sx - syaw * sy, self.y + syaw * sx + cy * sy, sz)

    def _place_on_deck(self, node, slot: int) -> None:
        """One-shot teleport of a package onto a deck slot (used at pickup).
        A SINGLE pose-set on a Newton dynamic body is safe; doing it every tick
        is not (it forces a per-step Newton resync that freezes the rover), so
        in-motion carry is a velocity servo instead — see _update_carry."""
        wx, wy, wz = self._deck_world(slot)
        node.getField("translation").setSFVec3f([wx, wy, wz])
        node.getField("rotation").setSFRotation([0, 0, 1, self.yaw])
        try:
            node.setVelocity([0.0] * 6)
        except Exception:
            pass

    def _update_carry(self) -> None:
        """Hold each carried package rigidly on its deck slot using VELOCITY
        only (no per-tick pose teleport): match the rover's body velocity plus
        a position-correction term that closes any drift to the slot. MuJoCo
        integrates the package along with the rover, so it rides cleanly."""
        if not self.carry:
            return
        try:
            rv = self.me.getVelocity()   # world [vx,vy,vz, wx,wy,wz]
        except Exception:
            rv = [0.0] * 6
        K = 8.0
        for c in self.carry:
            pk = self.packages.get(c["name"])
            node = pk["node"] if pk else None
            if node is None:
                continue
            tx, ty, tz = self._deck_world(c["slot"])
            pp = node.getPosition()
            try:
                node.setVelocity([rv[0] + K * (tx - pp[0]),
                                  rv[1] + K * (ty - pp[1]),
                                  K * (tz - pp[2]),
                                  0.0, 0.0, rv[5]])
            except Exception:
                pass

    # ── reset ─────────────────────────────────────────────────────
    def _do_reset(self) -> None:
        self._reset = False
        self.active = None
        self.phase = "idle"
        self.path = []
        self.carry = []
        self.delivered_at = {}
        self.me.setVelocity([0.0] * 6)
        self._mtf.setSFVec3f([self.spawn_xy[0], self.spawn_xy[1], 0.02])
        self._mrf.setSFRotation([0, 0, 1, self.spawn_yaw])
        try:
            self.me.resetPhysics()
        except Exception:
            pass
        for pk in self.packages.values():
            if pk["node"] is not None:
                pk["node"].getField("translation").setSFVec3f(list(pk["spawn"]))
                pk["node"].getField("rotation").setSFRotation([0, 0, 1, 0])
                try:
                    pk["node"].resetPhysics()
                    pk["node"].setVelocity([0.0] * 6)
                except Exception:
                    pass
        self.last_event = "reset to start"
