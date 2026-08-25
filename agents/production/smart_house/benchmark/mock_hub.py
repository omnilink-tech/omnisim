#!/usr/bin/env python3
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

"""mock_hub — an offline, contract-faithful smart-house hub.

Serves the same HTTP contract as the real ``smart_house_bridge`` controller
(the ``smart_house_bridge`` controller): every hub verb, the ``/scenario/*``
namespace, and the PROTOCOL.md conformance GETs — with a simplified linear
per-room temperature model instead of the engine. No simulator, no network,
no dependencies beyond the stdlib.

Two uses:

1. Offline dev server for anyone without the simulator (mirrors the
   omnilink-lib ``mock_bridge.py`` precedent)::

       python agents/production/smart_house/benchmark/mock_hub.py --port 8766

2. In-process test double for ``compare_tiers.py --mock`` and the test
   suite (``MockHub(port=0).start()``).

Model notes (assumptions the REAL bridge must roughly honour — see the
benchmark README):

* House time is DETERMINISTIC here: it advances ONLY via
  ``POST /scenario/advance`` (the real bridge free-runs with the engine).
* Per-room temperature: dT/dt = (T_out - T)/tau + sum(source gains),
  tau = 90 house-min, integrated in 1-minute Euler steps. The oven heats the
  kitchen at +0.36 C/min (20% spill to hallway); the heater heats every room
  at +0.20 C/min. Calibration matches the contract: oven left on takes an
  away-house kitchen from 20 C past 35 C within ~2 house-hours; the heater
  holds 21 C against 8 C outside.
* Outside temperature: sinusoid, 8 C at 04:00 to 14 C at 16:00.
* ``energy_spike`` fires only when total draw exceeds 3000 W for more than
  30 consecutive house-minutes while the resident is away — the oven alone
  (2430 W with standby) deliberately does NOT trip it, because inferring
  "oven left on" from device state is the agent-intelligence half of the
  demo (the contract says check_anomalies must not hand it over).
* ``set_device`` on ``door.front`` is ACCEPTED (motorized door assumption)
  so an agent can close a breached door remotely.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
from datetime import datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

ROOMS = ["living_room", "kitchen", "bedroom", "hallway"]
ROOM_NAMES = {
    "living_room": "Living Room",
    "kitchen": "Kitchen",
    "bedroom": "Bedroom",
    "hallway": "Hallway",
}

TAU_MIN = 90.0                 # thermal time constant, house-minutes
HEATER_GAIN_C_PER_MIN = 0.20   # 2.0 kW heater, every room
OVEN_KITCHEN_C_PER_MIN = 0.36  # 2.4 kW oven, kitchen
OVEN_SPILL_FRACTION = 0.20     # of the kitchen gain, into the hallway
STANDBY_W = 30.0
SPIKE_WATTS = 3000.0
SPIKE_SUSTAIN_MIN = 30
DEFAULT_START = "2026-01-05T08:00:00"
DEFAULT_TIME_SCALE = 60.0

FAULTS = (
    "device_offline",
    "state_rejected",
    "authorization_required",
    "degraded",
    "scene_unknown",
    "room_unknown",
)


def _fault(code: str, message: str) -> Dict[str, Any]:
    assert code in FAULTS, code
    return {"accepted": False, "error": code, "message": message}


def _default_devices() -> Dict[str, Dict[str, Any]]:
    def light(room: str, watts: int) -> Dict[str, Any]:
        return {
            "type": "light", "room": room, "watts": watts,
            "state": {"on": False, "brightness": 100},
            "capabilities": ["on", "off", "{on, brightness 0-100}"],
        }

    def switch(room: str, watts: int) -> Dict[str, Any]:
        return {
            "type": "switch", "room": room, "watts": watts,
            "state": "off", "capabilities": ["on", "off"],
        }

    return {
        "lights.living_ceiling": light("living_room", 60),
        "lights.living_lamp": light("living_room", 40),
        "lights.kitchen_ceiling": light("kitchen", 60),
        "lights.bedroom_ceiling": light("bedroom", 60),
        "lights.bedroom_lamp": light("bedroom", 40),
        "lights.hallway": light("hallway", 40),
        "appliance.oven": switch("kitchen", 2400),
        "appliance.coffee_maker": {
            **switch("kitchen", 900), "coffee_ready": False, "on_minutes": 0.0,
        },
        "appliance.tv": switch("living_room", 150),
        "thermostat.main": {
            "type": "thermostat", "room": "hallway", "watts": 0,
            "state": {"target": 21.0, "mode": "heat"},
            "capabilities": ["target 5-30 C", "mode heat|eco|off"],
        },
        "hvac.heater": {
            "type": "internal", "room": None, "watts": 2000,
            "state": "off", "capabilities": ["driven by thermostat hysteresis"],
        },
        "lock.front_door": {
            "type": "lock", "room": "hallway", "watts": 0,
            "state": "locked", "capabilities": ["locked", "unlocked (authorization)"],
        },
        "door.front": {
            "type": "door", "room": "hallway", "watts": 0,
            "state": "closed", "capabilities": ["open", "closed"],
        },
        "blinds.living": {
            "type": "cover", "room": "living_room", "watts": 0,
            "state": "open", "capabilities": ["open", "closed"],
        },
        "security.system": {
            "type": "alarm", "room": None, "watts": 0,
            "state": "disarmed", "capabilities": ["armed", "disarmed (authorization)"],
        },
    }


class HouseModel:
    """Deterministic linear-thermal smart house. All times are house time."""

    def __init__(self) -> None:
        self.reset()

    # -- lifecycle -----------------------------------------------------

    def reset(self) -> None:
        self.scenario_name: Optional[str] = None
        self.time_scale = DEFAULT_TIME_SCALE
        self.start_time = datetime.fromisoformat(DEFAULT_START)
        self.clock = self.start_time
        self.elapsed_min = 0.0
        self.resident: str = "living_room"          # room id or "away"
        self.devices = _default_devices()
        self.temps: Dict[str, float] = {r: 20.0 for r in ROOMS}
        self.meta: Dict[str, Dict[str, Any]] = {
            d: {"last_change": self._iso(), "changed_by": "hub"} for d in self.devices
        }
        self.energy_wh: Dict[str, float] = {d: 0.0 for d in self.devices}
        self.energy_wh["standby"] = 0.0
        self.device_log: List[Dict[str, Any]] = []
        self.notifications: List[Dict[str, Any]] = []
        self.temp_timeline: List[Dict[str, Any]] = []
        self.anomalies_active: Dict[str, Dict[str, Any]] = {}
        self.anomalies_history: List[Dict[str, Any]] = []
        self._spike_minutes = 0
        self._sample_countdown = 0
        self._sample_timeline()

    def start_scenario(self, name: str, seed: Optional[int] = None,
                       time_scale: Optional[float] = None,
                       start_time: Optional[str] = None) -> Dict[str, Any]:
        self.reset()
        self.scenario_name = name
        if time_scale:
            self.time_scale = float(time_scale)
        if start_time:
            # Extension over the frozen contract (see benchmark README):
            # "HH:MM" on the default date, or a full ISO timestamp.
            if len(start_time) <= 5 and ":" in start_time:
                base = datetime.fromisoformat(DEFAULT_START)
                h, m = start_time.split(":")
                self.start_time = base.replace(hour=int(h), minute=int(m))
            else:
                self.start_time = datetime.fromisoformat(start_time)
            self.clock = self.start_time
            self.temp_timeline = []
            self._sample_countdown = 0
            self._sample_timeline()
        return {
            "ok": True, "name": name, "house_time": self._iso(),
            "time_scale": self.time_scale,
        }

    # -- helpers --------------------------------------------------------

    def _iso(self) -> str:
        return self.clock.isoformat(timespec="seconds")

    def outside_temp(self) -> float:
        h = self.clock.hour + self.clock.minute / 60.0
        return 11.0 - 3.0 * math.cos(2.0 * math.pi * (h - 4.0) / 24.0)

    def _light_watts(self, dev: Dict[str, Any]) -> float:
        st = dev["state"]
        if isinstance(st, dict) and st.get("on"):
            return dev["watts"] * float(st.get("brightness", 100)) / 100.0
        return 0.0

    def _instant_watts(self) -> Dict[str, float]:
        draw: Dict[str, float] = {}
        for did, dev in self.devices.items():
            t = dev["type"]
            if t == "light":
                w = self._light_watts(dev)
            elif t == "switch":
                w = dev["watts"] if dev["state"] == "on" else 0.0
            elif t == "internal":
                w = dev["watts"] if dev["state"] == "on" else 0.0
            else:
                w = 0.0
            if w:
                draw[did] = w
        draw["standby"] = STANDBY_W
        return draw

    def _log_device(self, did: str, state: Any, changed_by: str) -> None:
        self.meta[did] = {"last_change": self._iso(), "changed_by": changed_by}
        self.device_log.append({
            "house_time": self._iso(), "id": did,
            "state": json.loads(json.dumps(state)), "changed_by": changed_by,
        })

    def _sample_timeline(self) -> None:
        self.temp_timeline.append({
            "house_time": self._iso(),
            "temps": {r: round(t, 3) for r, t in self.temps.items()},
        })

    # -- physics --------------------------------------------------------

    def advance(self, house_minutes: float) -> Dict[str, Any]:
        whole = int(round(house_minutes))
        for _ in range(max(0, whole)):
            self._step_one_minute()
        return {
            "ok": True, "house_time": self._iso(),
            "advanced_house_min": whole, "elapsed_house_min": self.elapsed_min,
        }

    def _step_one_minute(self) -> None:
        tout = self.outside_temp()
        thermo = self.devices["thermostat.main"]["state"]
        heater = self.devices["hvac.heater"]

        # Thermostat hysteresis (+-0.5 C) on the hallway temperature.
        mode = thermo.get("mode", "off")
        eff_target: Optional[float] = None
        if mode == "heat":
            eff_target = float(thermo.get("target", 21.0))
        elif mode == "eco":
            eff_target = float(thermo.get("target", 21.0)) - 4.0
        hall_t = self.temps["hallway"]
        if eff_target is None:
            if heater["state"] == "on":
                heater["state"] = "off"
                self._log_device("hvac.heater", "off", "hub")
        else:
            if heater["state"] == "off" and hall_t < eff_target - 0.5:
                heater["state"] = "on"
                self._log_device("hvac.heater", "on", "hub")
            elif heater["state"] == "on" and hall_t > eff_target + 0.5:
                heater["state"] = "off"
                self._log_device("hvac.heater", "off", "hub")

        # Heat sources.
        gains = {r: 0.0 for r in ROOMS}
        if heater["state"] == "on":
            for r in ROOMS:
                gains[r] += HEATER_GAIN_C_PER_MIN
        if self.devices["appliance.oven"]["state"] == "on":
            gains["kitchen"] += OVEN_KITCHEN_C_PER_MIN
            gains["hallway"] += OVEN_KITCHEN_C_PER_MIN * OVEN_SPILL_FRACTION

        # Temperature integration (1-minute Euler).
        for r in ROOMS:
            self.temps[r] += (tout - self.temps[r]) / TAU_MIN + gains[r]

        # Energy integration.
        draw = self._instant_watts()
        for did, w in draw.items():
            self.energy_wh[did] = self.energy_wh.get(did, 0.0) + w / 60.0

        # Coffee maker timer.
        coffee = self.devices["appliance.coffee_maker"]
        if coffee["state"] == "on":
            coffee["on_minutes"] += 1.0
            if coffee["on_minutes"] >= 5.0 and not coffee["coffee_ready"]:
                coffee["coffee_ready"] = True

        # Clock.
        self.clock += timedelta(minutes=1)
        self.elapsed_min += 1.0

        # Anomaly tracking.
        self._update_anomalies(sum(draw.values()))

        # Timeline sample every 5 house-min.
        self._sample_countdown -= 1
        if self._sample_countdown <= 0:
            self._sample_timeline()
            self._sample_countdown = 5

    def _update_anomalies(self, total_watts: float) -> None:
        door_open = self.devices["door.front"]["state"] == "open"
        armed = self.devices["security.system"]["state"] == "armed"
        self._set_anomaly(
            "door_open_while_armed", door_open and armed,
            {"id": "door.front", "details": "front door is open while the security system is armed"},
        )
        if total_watts > SPIKE_WATTS and self.resident == "away":
            self._spike_minutes += 1
        else:
            self._spike_minutes = 0
        self._set_anomaly(
            "energy_spike", self._spike_minutes > SPIKE_SUSTAIN_MIN,
            {"details": f"total draw above {SPIKE_WATTS:.0f} W for over "
                        f"{SPIKE_SUSTAIN_MIN} house-min while away"},
        )

    def _set_anomaly(self, atype: str, active: bool, fields: Dict[str, Any]) -> None:
        cur = self.anomalies_active.get(atype)
        if active and cur is None:
            self.anomalies_active[atype] = {
                "type": atype, "since": self._iso(), **fields,
            }
        elif not active and cur is not None:
            done = dict(cur)
            done["resolved_at"] = self._iso()
            self.anomalies_history.append(done)
            del self.anomalies_active[atype]

    # -- hub verbs -------------------------------------------------------

    def list_rooms(self) -> List[Dict[str, Any]]:
        return [
            {
                "id": r, "name": ROOM_NAMES[r],
                "device_count": sum(1 for d in self.devices.values() if d["room"] == r),
            }
            for r in ROOMS
        ]

    def list_devices(self, room: Optional[str] = None) -> Any:
        if room is not None and room not in ROOMS:
            return _fault("room_unknown", f"unknown room id: {room!r}")
        out = []
        for did, dev in self.devices.items():
            if room and dev["room"] != room:
                continue
            entry = {
                "id": did, "type": dev["type"], "room_id": dev["room"],
                "state": dev["state"], "capabilities": dev["capabilities"],
            }
            if did == "appliance.coffee_maker":
                entry["coffee_ready"] = dev["coffee_ready"]
            out.append(entry)
        return out

    def read_sensors(self, room: Optional[str] = None) -> Dict[str, Any]:
        if room is not None and room not in ROOMS + ["outside"]:
            return _fault("room_unknown", f"unknown room id: {room!r}")
        ts = self._iso()
        readings: List[Dict[str, Any]] = []
        for r in ROOMS:
            if room and r != room:
                continue
            readings.append({"room_id": r, "type": "temperature",
                             "value": round(self.temps[r], 2), "unit": "C",
                             "timestamp": ts})
            readings.append({"room_id": r, "type": "motion",
                             "value": (self.resident == r), "timestamp": ts})
        if room in (None, "outside"):
            readings.append({"room_id": "outside", "type": "temperature",
                             "value": round(self.outside_temp(), 2), "unit": "C",
                             "timestamp": ts})
        return {"readings": readings}

    def get_device_state(self, did: str) -> Dict[str, Any]:
        dev = self.devices.get(did)
        if dev is None:
            return _fault("device_offline", f"unknown device id: {did!r}")
        meta = self.meta.get(did, {})
        out: Dict[str, Any] = {
            "id": did, "state": dev["state"],
            "last_change": meta.get("last_change"),
            "changed_by": meta.get("changed_by"),
            "online": True,
        }
        if dev["type"] == "thermostat":
            out["mode"] = dev["state"].get("mode")
        if did == "appliance.coffee_maker":
            out["coffee_ready"] = dev["coffee_ready"]
        return out

    def set_device(self, did: str, state: Any,
                   changed_by: str = "agent",
                   authorization: str = "") -> Dict[str, Any]:
        dev = self.devices.get(did)
        if dev is None:
            return _fault("device_offline", f"unknown device id: {did!r}")
        t = dev["type"]
        if t == "light":
            if state == "on":
                dev["state"]["on"] = True
            elif state == "off":
                dev["state"]["on"] = False
            elif isinstance(state, dict):
                if "on" in state:
                    dev["state"]["on"] = bool(state["on"])
                if "brightness" in state:
                    try:
                        b = max(0, min(100, int(state["brightness"])))
                    except (TypeError, ValueError):
                        return _fault("state_rejected", "brightness must be 0-100")
                    dev["state"]["brightness"] = b
            else:
                return _fault("state_rejected",
                              "lights accept 'on', 'off', or {on, brightness}")
        elif t == "switch":
            if state not in ("on", "off"):
                return _fault("state_rejected", "switches accept 'on' or 'off'")
            if did == "appliance.coffee_maker" and state == "on" and dev["state"] != "on":
                dev["on_minutes"] = 0.0
                dev["coffee_ready"] = False
            dev["state"] = state
        elif t in ("cover", "door"):
            if state not in ("open", "closed"):
                return _fault("state_rejected", f"{t} accepts 'open' or 'closed'")
            dev["state"] = state
        elif t == "thermostat":
            if not isinstance(state, dict):
                return _fault("state_rejected",
                              "thermostat accepts {target?, mode?} — or use adjust_thermostat")
            return self.adjust_thermostat(did, state.get("target"),
                                          state.get("mode"), changed_by)
        elif t == "lock":
            if state == "locked":
                dev["state"] = "locked"
            elif state == "unlocked":
                if not authorization:
                    return _fault("authorization_required",
                                  "unlocking needs an occupant authorization token")
                dev["state"] = "unlocked"
            else:
                return _fault("state_rejected", "locks accept 'locked' or 'unlocked'")
        elif t == "alarm":
            if state == "armed":
                dev["state"] = "armed"
            elif state == "disarmed":
                if not authorization:
                    return _fault("authorization_required",
                                  "disarming needs an occupant authorization token")
                dev["state"] = "disarmed"
            else:
                return _fault("state_rejected", "alarm accepts 'armed' or 'disarmed'")
        elif t == "internal":
            return _fault("state_rejected",
                          "hvac.heater is driven by the thermostat — use adjust_thermostat")
        else:
            return _fault("state_rejected", f"cannot set device of type {t!r}")
        self._log_device(did, dev["state"], changed_by)
        return {"accepted": True, "realized_state": dev["state"]}

    def toggle_device(self, did: str) -> Dict[str, Any]:
        dev = self.devices.get(did)
        if dev is None:
            return _fault("device_offline", f"unknown device id: {did!r}")
        if dev["type"] == "light":
            res = self.set_device(did, "off" if dev["state"]["on"] else "on")
        elif dev["type"] == "switch":
            res = self.set_device(did, "off" if dev["state"] == "on" else "on")
        else:
            return _fault("state_rejected",
                          f"toggle_device only handles lights and switches, not {dev['type']!r}")
        if not res.get("accepted"):
            return res
        return {"accepted": True, "new_state": res["realized_state"]}

    def set_scene(self, scene: str) -> Dict[str, Any]:
        plans: Dict[str, List[Tuple[str, Any]]] = {
            "morning": [
                ("blinds.living", "open"),
                ("thermostat.main", {"target": 21, "mode": "heat"}),
                ("lights.kitchen_ceiling", {"on": True, "brightness": 60}),
                ("appliance.coffee_maker", "on"),
            ],
            "goodnight": [
                ("lights.living_ceiling", "off"), ("lights.living_lamp", "off"),
                ("lights.kitchen_ceiling", "off"), ("lights.bedroom_ceiling", "off"),
                ("lights.hallway", "off"),
                ("lights.bedroom_lamp", {"on": True, "brightness": 10}),
                ("blinds.living", "closed"),
                ("thermostat.main", {"target": 17, "mode": "heat"}),
                ("security.system", "armed"),
                ("lock.front_door", "locked"),
            ],
            "movie": [
                ("lights.living_ceiling", {"on": True, "brightness": 10}),
                ("lights.living_lamp", {"on": True, "brightness": 10}),
                ("blinds.living", "closed"),
                ("appliance.tv", "on"),
            ],
            "away": [
                ("thermostat.main", {"mode": "eco"}),
                ("lights.living_ceiling", "off"), ("lights.living_lamp", "off"),
                ("lights.kitchen_ceiling", "off"), ("lights.bedroom_ceiling", "off"),
                ("lights.bedroom_lamp", "off"), ("lights.hallway", "off"),
                ("security.system", "armed"),
                ("lock.front_door", "locked"),
            ],
        }
        plan = plans.get(scene)
        if plan is None:
            return _fault("scene_unknown",
                          f"unknown scene {scene!r}; scenes: {sorted(plans)}")
        affected = []
        for did, state in plan:
            res = self.set_device(did, state, changed_by="agent")
            if res.get("accepted"):
                realized = res.get("realized_state")
                if realized is None:  # adjust_thermostat path: {target, mode}
                    realized = json.loads(json.dumps(self.devices[did]["state"]))
                affected.append({"id": did, "state": realized})
        return {"accepted": True, "affected": affected}

    def adjust_thermostat(self, did: str, target: Optional[float],
                          mode: Optional[str],
                          changed_by: str = "agent") -> Dict[str, Any]:
        if did != "thermostat.main":
            return _fault("device_offline", f"unknown thermostat id: {did!r}")
        st = self.devices[did]["state"]
        clamped = False
        if target is not None:
            try:
                tv = float(target)
            except (TypeError, ValueError):
                return _fault("state_rejected", "target must be a number")
            if tv < 5.0:
                tv, clamped = 5.0, True
            elif tv > 30.0:
                tv, clamped = 30.0, True
            st["target"] = tv
        if mode is not None:
            if mode not in ("heat", "eco", "off"):
                return _fault("state_rejected", "mode must be heat, eco, or off")
            st["mode"] = mode
        self._log_device(did, dict(st), changed_by)
        out = {"accepted": True, "target": st["target"], "mode": st["mode"]}
        if clamped:
            out["clamped"] = True
        return out

    def lock_door(self, did: str) -> Dict[str, Any]:
        if did not in ("lock.front_door", "door.front"):
            return _fault("device_offline", f"unknown lock id: {did!r}")
        self.devices["lock.front_door"]["state"] = "locked"
        self._log_device("lock.front_door", "locked", "agent")
        return {"accepted": True, "state": "locked"}

    def unlock_door(self, did: str, authorization: str) -> Dict[str, Any]:
        if did not in ("lock.front_door", "door.front"):
            return _fault("device_offline", f"unknown lock id: {did!r}")
        if not authorization:
            return _fault("authorization_required",
                          "unlock_door needs an occupant authorization token")
        self.devices["lock.front_door"]["state"] = "unlocked"
        self._log_device("lock.front_door", "unlocked", "agent")
        return {"accepted": True, "state": "unlocked"}

    def arm_security(self, zone: Optional[str] = None) -> Dict[str, Any]:
        self.devices["security.system"]["state"] = "armed"
        self._log_device("security.system", "armed", "agent")
        out: Dict[str, Any] = {"accepted": True, "state": "armed"}
        if zone:
            out["zone"] = zone
        return out

    def disarm_security(self, zone: Optional[str],
                        authorization: str) -> Dict[str, Any]:
        if not authorization:
            return _fault("authorization_required",
                          "disarm_security needs an occupant authorization token")
        self.devices["security.system"]["state"] = "disarmed"
        self._log_device("security.system", "disarmed", "agent")
        out: Dict[str, Any] = {"accepted": True, "state": "disarmed"}
        if zone:
            out["zone"] = zone
        return out

    def get_energy_report(self, range_: str = "24h") -> Dict[str, Any]:
        cats = {"hvac": 0.0, "lighting": 0.0, "appliances": 0.0, "standby": 0.0}
        for did, wh in self.energy_wh.items():
            if did == "hvac.heater":
                cats["hvac"] += wh
            elif did.startswith("lights."):
                cats["lighting"] += wh
            elif did.startswith("appliance."):
                cats["appliances"] += wh
            elif did == "standby":
                cats["standby"] += wh
        total_wh = sum(self.energy_wh.values())
        outliers = []
        for did, wh in self.energy_wh.items():
            if did != "standby" and total_wh > 0 and wh / total_wh > 0.4:
                outliers.append({"id": did, "wh": round(wh, 1),
                                 "share": round(wh / total_wh, 3)})
        return {
            "total_kwh": round(total_wh / 1000.0, 4),
            "by_category": {k: round(v / 1000.0, 4) for k, v in cats.items()},
            "outliers": outliers,
            "range": range_,
            "range_applied": "scenario_start",
        }

    def check_anomalies(self) -> Dict[str, Any]:
        return {
            "active": list(self.anomalies_active.values()),
            "history": list(self.anomalies_history),
        }

    def get_weather(self, location: Optional[str] = None) -> Dict[str, Any]:
        return {
            "temp_c": round(self.outside_temp(), 2),
            "condition": "clear",
            "house_time": self._iso(),
        }

    def notify_occupant(self, message: str, severity: str = "medium",
                        channel: Optional[str] = None,
                        occupant_id: Optional[str] = None) -> Dict[str, Any]:
        rec = {
            "house_time": self._iso(), "message": message,
            "severity": severity, "channel": channel or "in_app_sim",
        }
        if occupant_id:
            rec["occupant_id"] = occupant_id
        self.notifications.append(rec)
        return {"delivered": True, "channel": rec["channel"]}

    # -- scenario --------------------------------------------------------

    def scenario_status(self) -> Dict[str, Any]:
        return {
            "name": self.scenario_name, "house_time": self._iso(),
            "phase": "running" if self.scenario_name else "idle",
            "resident": self.resident, "elapsed_house_min": self.elapsed_min,
        }

    def scenario_resident(self, where: str) -> Dict[str, Any]:
        if where != "away" and where not in ROOMS:
            return _fault("room_unknown", f"unknown room id: {where!r}")
        self.resident = where
        return {"ok": True, "resident": self.resident}

    def scenario_event(self, etype: str, did: str, state: Any) -> Dict[str, Any]:
        if etype == "door":
            did = did or "door.front"
        if etype not in ("device_set", "door"):
            return _fault("state_rejected", f"unknown event type {etype!r}")
        res = self.set_device(did, state, changed_by="scenario",
                              authorization="scenario_actor")
        if not res.get("accepted"):
            return res
        return {"ok": True, "id": did, "state": res["realized_state"],
                "changed_by": "scenario"}

    def metrics(self) -> Dict[str, Any]:
        return {
            "energy_wh_total": round(sum(self.energy_wh.values()), 1),
            "energy_wh_by_device": {k: round(v, 1) for k, v in self.energy_wh.items() if v > 0},
            "room_temps": {r: round(t, 2) for r, t in self.temps.items()},
            "temp_timeline": list(self.temp_timeline),
            "notifications": list(self.notifications),
            "device_log": list(self.device_log),
            "anomalies_history": list(self.anomalies_history)
                                 + list(self.anomalies_active.values()),
            "house_time": self._iso(),
        }

    def stop_robot(self) -> Dict[str, Any]:
        self.set_device("appliance.oven", "off", changed_by="hub")
        self.set_device("appliance.coffee_maker", "off", changed_by="hub")
        self.adjust_thermostat("thermostat.main", None, "off", changed_by="hub")
        return {"ok": True, "halted_at": self._iso()}


# ---------------------------------------------------------------------------
# HTTP layer
# ---------------------------------------------------------------------------

def _make_handler(model: HouseModel, lock: threading.RLock):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # noqa: ARG002
            pass

        def _send(self, status: int, payload: Any) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            with lock:
                if path == "/healthz":
                    return self._send(200, {"ok": True, "service": "mock_hub"})
                if path == "/protocol":
                    return self._send(200, {
                        "ok": True, "omnisim_wire": "1.0",
                        "service": "robot_bridge",
                        "service_versions": {"mock_hub": "1.0"},
                        "instance": {"name": "smart_house",
                                     "robot_id": "smart_house", "mock": True},
                    })
                if path == "/capabilities":
                    return self._send(200, self._capabilities())
                if path == "/state":
                    return self._send(200, {
                        "ok": True, "robot_id": "smart_house",
                        "sim_time": model.elapsed_min * 60.0 / model.time_scale,
                        "last_tick_at": model._iso(), "mode": "mock",
                        "fault": None, "state_source": "mock_model",
                    })
                if path == "/read_mission_brief":
                    return self._send(200, {
                        "ok": True,
                        "brief": "Manage the OmniLink smart house: comfort when "
                                 "occupied, economy and security when away.",
                    })
            self._send(404, {"ok": False, "error": "unknown_endpoint"})

        def _capabilities(self) -> Dict[str, Any]:
            return {
                "ok": True,
                "robots": [{
                    "id": "smart_house", "robot_id": "smart_house",
                    "model": "OmniLink Smart House", "class": "smart_home_hub",
                    "actions": [
                        "list_rooms", "list_devices", "read_sensors",
                        "get_device_state", "check_anomalies",
                        "get_energy_report", "get_weather", "set_device",
                        "toggle_device", "set_scene", "adjust_thermostat",
                        "set_schedule", "lock_door", "unlock_door",
                        "arm_security", "disarm_security", "shut_water_main",
                        "shut_gas_main", "notify_occupant",
                    ],
                    "sensors": ["temperature", "motion", "outside_temperature"],
                }],
            }

        def do_POST(self):  # noqa: N802
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b""
            if raw.strip():
                try:
                    body = json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError:
                    return self._send(400, {"ok": False, "error": "bad_json"})
                if not isinstance(body, dict):
                    return self._send(400, {"ok": False, "error": "bad_json"})
            else:
                body = {}
            with lock:
                handled, payload = self._route(path.lstrip("/"), body)
            if handled:
                return self._send(200, payload)
            self._send(404, {"ok": False, "error": "unknown_endpoint"})

        def _route(self, ep: str, b: Dict[str, Any]):
            m = model
            table = {
                "list_rooms": lambda: m.list_rooms(),
                "list_devices": lambda: m.list_devices(b.get("room")),
                "read_sensors": lambda: m.read_sensors(b.get("room")),
                "get_device_state": lambda: m.get_device_state(b.get("id", "")),
                "set_device": lambda: m.set_device(
                    b.get("id", ""), b.get("state"),
                    authorization=b.get("authorization", "")),
                "toggle_device": lambda: m.toggle_device(b.get("id", "")),
                "set_scene": lambda: m.set_scene(b.get("scene", "")),
                "adjust_thermostat": lambda: m.adjust_thermostat(
                    b.get("id", ""), b.get("target"), b.get("mode")),
                "set_schedule": lambda: {
                    "accepted": False, "error": "state_rejected",
                    "message": "this hub does not execute device schedules"},
                "lock_door": lambda: m.lock_door(b.get("id", "")),
                "unlock_door": lambda: m.unlock_door(
                    b.get("id", ""), b.get("authorization", "")),
                "arm_security": lambda: m.arm_security(b.get("zone")),
                "disarm_security": lambda: m.disarm_security(
                    b.get("zone"), b.get("authorization", "")),
                "get_energy_report": lambda: m.get_energy_report(b.get("range", "24h")),
                "check_anomalies": lambda: m.check_anomalies(),
                "get_weather": lambda: m.get_weather(b.get("location")),
                "notify_occupant": lambda: m.notify_occupant(
                    b.get("message", ""), b.get("severity", "medium"),
                    b.get("channel"), b.get("occupant_id")),
                "shut_water_main": lambda: {
                    "accepted": False, "error": "state_rejected",
                    "message": "not plumbed in this house"},
                "shut_gas_main": lambda: {
                    "accepted": False, "error": "state_rejected",
                    "message": "not plumbed in this house"},
                "scenario/start": lambda: m.start_scenario(
                    b.get("name", "adhoc"), b.get("seed"),
                    b.get("time_scale"), b.get("start_time")),
                "scenario/status": lambda: m.scenario_status(),
                "scenario/resident": lambda: m.scenario_resident(
                    b.get("room", b.get("resident", ""))),
                "scenario/event": lambda: m.scenario_event(
                    b.get("type", "device_set"), b.get("id", ""), b.get("state")),
                "scenario/metrics": lambda: m.metrics(),
                "scenario/advance": lambda: m.advance(
                    float(b.get("house_minutes", 0))),
                "scenario/reset": lambda: (m.reset(), {"ok": True})[1],
                "stop_robot": lambda: m.stop_robot(),
                "reset_to_home": lambda: (m.reset(), {"ok": True})[1],
                "list_robots": lambda: self._capabilities(),
                "get_robot_state": lambda: {
                    "ok": True, "robot_id": "smart_house",
                    "sim_time": m.elapsed_min * 60.0 / m.time_scale,
                    "last_tick_at": m._iso(), "mode": "mock",
                    "fault": None, "state_source": "mock_model"},
            }
            fn = table.get(ep)
            if fn is None:
                return False, None
            return True, fn()

    return Handler


class MockHub:
    """In-process mock hub: ``hub = MockHub().start(); ... hub.stop()``."""

    def __init__(self, port: int = 0) -> None:
        self.model = HouseModel()
        self.lock = threading.RLock()
        self._requested_port = port
        self.server: Optional[ThreadingHTTPServer] = None
        self.port: int = 0

    def start(self) -> "MockHub":
        handler = _make_handler(self.model, self.lock)
        self.server = ThreadingHTTPServer(("127.0.0.1", self._requested_port), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        return self

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def stop(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.server = None


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline smart-house hub (mock).")
    ap.add_argument("--port", type=int, default=8766)
    args = ap.parse_args()
    hub = MockHub(port=args.port).start()
    print(f"mock_hub serving the smart-house contract on {hub.url}")
    print("POST hub verbs + /scenario/*; GET /healthz /protocol /capabilities /state")
    print("Ctrl+C to stop.")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        hub.stop()


if __name__ == "__main__":
    main()
