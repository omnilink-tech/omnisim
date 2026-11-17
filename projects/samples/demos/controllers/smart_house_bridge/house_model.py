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

"""The smart-house model: clock, thermal model, energy meter, devices,
thermostat automation, anomaly engine and scenario engine.

Pure stdlib and pure state — no ``omnisim`` import, no HTTP, no threads.
The bridge (smart_house_bridge.py) owns the Supervisor and hands this
model a *scene* object implementing the small SceneInterface below; the
test suite hands it a fake. Every device state change flows through
``_commit`` so the device log, ``changed_by`` bookkeeping and the visual
side effect can never drift apart.

Units
-----
- House time is minutes since midnight of BASE_DATE ("house-min", hmin).
- ``advance(sim_now_s)`` converts simulator seconds to house minutes with
  ``time_scale`` (default 60: one sim second = one house minute).
- Temperatures are Celsius; energy is watt-hours integrated in HOUSE time.

Thermal model (contract calibration)
------------------------------------
Per room:  dT/dt = (T_out - T)/TAU + sum(source rates)   [degC/house-min]
- TAU = 90 house-min.
- Oven on: +0.36 degC/hmin in the kitchen, +0.072 in the hallway (20%
  spill). From 20 degC with T_out ~10 degC that reaches ~36 degC in 120
  house-min (the "oven left on -> 35 degC+ within ~2 house-hours" spec).
- Heater on: +0.25 degC/hmin in every room. Steady loss at 21 degC vs
  8 degC outside is (8-21)/90 = -0.144 degC/hmin, so the heater holds
  21 degC with ~58% duty.
- Outside: sinusoid 8 degC (04:00) to 14 degC (16:00), overridable.
"""

from __future__ import annotations

import datetime as _dt
from typing import Any, Callable, Dict, List, Optional, Tuple

# ── Constants ─────────────────────────────────────────────────────────

BASE_DATE = _dt.datetime(2026, 8, 19)  # day 1 of house time

ROOMS = ["living_room", "kitchen", "bedroom", "hallway"]
ROOM_NAMES = {
    "living_room": "Living Room",
    "kitchen": "Kitchen",
    "bedroom": "Bedroom",
    "hallway": "Hallway",
}
# World-frame bounds (xmin, xmax, ymin, ymax) — used to derive the motion
# sensor / measured resident room from the RESIDENT node's actual position.
ROOM_BOUNDS = {
    "living_room": (-5.0, 0.0, 0.0, 4.0),
    "kitchen": (0.0, 5.0, 0.0, 4.0),
    "hallway": (-5.0, 0.0, -4.0, 0.0),
    "bedroom": (0.0, 5.0, -4.0, 0.0),
}

LIGHTS = {
    "lights.living_ceiling": {"room": "living_room", "watts": 60.0},
    "lights.living_lamp": {"room": "living_room", "watts": 40.0},
    "lights.kitchen_ceiling": {"room": "kitchen", "watts": 60.0},
    "lights.bedroom_ceiling": {"room": "bedroom", "watts": 60.0},
    "lights.bedroom_lamp": {"room": "bedroom", "watts": 40.0},
    "lights.hallway": {"room": "hallway", "watts": 40.0},
}
SWITCHES = {
    "appliance.oven": {"room": "kitchen", "watts": 2400.0},
    "appliance.coffee_maker": {"room": "kitchen", "watts": 900.0},
    "appliance.tv": {"room": "living_room", "watts": 150.0},
}
HEATER_WATTS = 2000.0
STANDBY_WATTS = 30.0

ALL_DEVICE_IDS = (
    list(LIGHTS) + list(SWITCHES)
    + ["thermostat.main", "hvac.heater", "lock.front_door", "door.front",
       "blinds.living", "security.system"]
)

DEVICE_TYPE = {}
DEVICE_TYPE.update({i: "light" for i in LIGHTS})
DEVICE_TYPE.update({i: "switch" for i in SWITCHES})
DEVICE_TYPE.update({
    "thermostat.main": "thermostat",
    "hvac.heater": "internal",
    "lock.front_door": "lock",
    "door.front": "door",
    "blinds.living": "cover",
    "security.system": "alarm",
})
DEVICE_ROOM = {}
DEVICE_ROOM.update({i: LIGHTS[i]["room"] for i in LIGHTS})
DEVICE_ROOM.update({i: SWITCHES[i]["room"] for i in SWITCHES})
DEVICE_ROOM.update({
    "thermostat.main": "hallway",
    "hvac.heater": None,
    "lock.front_door": "hallway",
    "door.front": "hallway",
    "blinds.living": "living_room",
    "security.system": None,
})

# Thermal calibration (degC per house-minute)
TAU_HMIN = 90.0
RATE_OVEN_KITCHEN = 0.36
RATE_OVEN_HALLWAY = 0.072
RATE_HEATER = 0.25
HYSTERESIS_C = 0.5
ECO_SETBACK_C = 4.0
COFFEE_READY_HMIN = 5.0

# Anomaly engine. energy_spike is deliberately NOT trippable by the oven
# alone (2400 W + 30 W standby = 2430 W < 3000 W): inferring "the oven was
# left on" from device state is the agent-intelligence part of the demo.
# The spike needs > 3000 W sustained > 30 house-min while the occupant is
# away (contract update 2026-08-19 #3).
ENERGY_SPIKE_WATTS = 3000.0
ENERGY_SPIKE_SUSTAIN_HMIN = 30.0

TIMELINE_SAMPLE_HMIN = 5.0
TIMELINE_MAX_SAMPLES = 4000
LOG_MAX_ENTRIES = 5000

FAULTS = ("device_offline", "state_rejected", "authorization_required",
          "degraded", "scene_unknown", "room_unknown")

SCENES = {
    "morning": [
        ("blinds.living", "open"),
        ("thermostat.main", {"target": 21.0, "mode": "heat"}),
        ("lights.kitchen_ceiling", {"on": True, "brightness": 60}),
        ("appliance.coffee_maker", "on"),
    ],
    "goodnight": [
        ("lights.living_ceiling", "off"),
        ("lights.living_lamp", "off"),
        ("lights.kitchen_ceiling", "off"),
        ("lights.bedroom_ceiling", "off"),
        ("lights.hallway", "off"),
        ("lights.bedroom_lamp", {"on": True, "brightness": 10}),
        ("blinds.living", "closed"),
        ("thermostat.main", {"target": 17.0}),
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
        ("lights.living_ceiling", "off"),
        ("lights.living_lamp", "off"),
        ("lights.kitchen_ceiling", "off"),
        ("lights.bedroom_ceiling", "off"),
        ("lights.bedroom_lamp", "off"),
        ("lights.hallway", "off"),
        ("security.system", "armed"),
        ("lock.front_door", "locked"),
    ],
}

# Scripted timelines: (offset_house_min, event). Events use the same shape
# as POST /scenario/event; "resident" moves the occupant prop. All scripted
# state changes land with changed_by="scenario".
# Canonical start times (contract update 2026-08-19 #1): s1 19:00,
# s2 08:00, s3 20:00 (door breach at 02:10 = +370), s4 21:00 (return
# 07:30 = +630). /scenario/start may override with start_time.
SCENARIOS = {
    "s1_movie_night": {
        "start_hm": (19, 0),
        "resident": "living_room",
        "timeline": [
            (180.0, {"type": "resident", "room": "bedroom"}),
        ],
    },
    "s2_oven_left_on": {
        "start_hm": (8, 0),
        "resident": "kitchen",
        "timeline": [
            (5.0, {"type": "device_set", "id": "appliance.oven", "state": "on"}),
            (15.0, {"type": "resident", "room": "hallway"}),
            (17.0, {"type": "door", "id": "door.front", "state": "open"}),
            (18.0, {"type": "door", "id": "door.front", "state": "closed"}),
            (18.0, {"type": "resident", "room": "away"}),
        ],
    },
    "s3_night_door": {
        "start_hm": (20, 0),
        "resident": "bedroom",  # asleep at home, NOT away
        "timeline": [
            (150.0, {"type": "device_set", "id": "security.system", "state": "armed"}),
            (150.0, {"type": "device_set", "id": "lock.front_door", "state": "locked"}),
            # 02:10, not 02:00: hourly wakes land on :00, and an incident
            # that coincides with a wake reads as instant detection —
            # mid-gap staging keeps the "cadence bounds detection" claim
            # honest (and matches the benchmark driver + mock hub).
            (370.0, {"type": "device_set", "id": "lock.front_door", "state": "unlocked"}),
            (370.0, {"type": "door", "id": "door.front", "state": "open"}),
        ],
    },
    "s4_morning_prep": {
        "start_hm": (21, 0),
        "resident": "away",
        "timeline": [
            (630.0, {"type": "door", "id": "door.front", "state": "open"}),
            (630.0, {"type": "resident", "room": "hallway"}),
            (632.0, {"type": "door", "id": "door.front", "state": "closed"}),
            (635.0, {"type": "resident", "room": "kitchen"}),
        ],
    },
}


def parse_start_time(value: Any) -> Optional[float]:
    """'HH:MM' or an ISO datetime -> house-minutes since BASE_DATE
    midnight, or None when unparseable."""
    if not isinstance(value, str) or not value.strip():
        return None
    text = value.strip()
    try:
        if ":" in text and len(text) <= 5:
            h, m = text.split(":", 1)
            h_i, m_i = int(h), int(m)
            if 0 <= h_i < 24 and 0 <= m_i < 60:
                return h_i * 60.0 + m_i
            return None
        dt = _dt.datetime.fromisoformat(text)
        if dt.tzinfo is not None:
            dt = dt.replace(tzinfo=None)
        return (dt - BASE_DATE).total_seconds() / 60.0
    except ValueError:
        return None


def fault(code: str, message: str) -> Dict[str, Any]:
    """Standardized command rejection (HTTP 200 body, Haven fault codes)."""
    assert code in FAULTS, code
    return {"accepted": False, "error": code, "message": message}


def outside_temp_c(clock_hmin: float) -> float:
    """Sinusoid: 8 degC at 04:00, 14 degC at 16:00."""
    import math
    hours = (clock_hmin / 60.0) % 24.0
    return 11.0 - 3.0 * math.cos(2.0 * math.pi * (hours - 4.0) / 24.0)


def house_time_iso(clock_hmin: float) -> str:
    return (BASE_DATE + _dt.timedelta(minutes=clock_hmin)).isoformat(timespec="seconds")


def room_from_position(x: float, y: float, z: float) -> str:
    """Map a world position to a room id; anything outside is 'away'."""
    if z < -1.0:
        return "away"
    for room, (x0, x1, y0, y1) in ROOM_BOUNDS.items():
        if x0 <= x < x1 and y0 <= y < y1:
            return room
    return "away"


class SceneInterface:
    """What the model needs from the world. The bridge implements this on
    the Supervisor; tests implement it on dicts. apply_* are fire-and-
    forget visual side effects; read_* are measurements."""

    def apply_light(self, dev_id: str, on: bool, brightness: float) -> None: ...
    def apply_switch(self, dev_id: str, on: bool) -> None: ...
    def apply_heater(self, on: bool) -> None: ...
    def apply_door(self, is_open: bool) -> None: ...
    def apply_blinds(self, is_open: bool) -> None: ...
    def apply_security(self, armed: bool) -> None: ...
    def apply_resident(self, room_or_away: str) -> None: ...

    def read_light(self, dev_id: str) -> Optional[Dict[str, Any]]: return None
    def read_switch(self, dev_id: str) -> Optional[bool]: return None
    def read_heater(self) -> Optional[bool]: return None
    def read_door_open(self) -> Optional[bool]: return None
    def read_blinds_open(self) -> Optional[bool]: return None
    def read_resident_position(self) -> Optional[Tuple[float, float, float]]: return None

    def is_online(self, dev_id: str) -> bool: return True


class HouseModel:
    """Authoritative device registry + physics-side house model."""

    def __init__(self, scene: SceneInterface, time_scale: float = 60.0):
        self.scene = scene
        self.time_scale = float(time_scale)
        self._last_sim_t: Optional[float] = None
        self.reset(clock_hmin=8 * 60.0)

    # ── Lifecycle ────────────────────────────────────────────────────

    def reset(self, clock_hmin: float = 8 * 60.0) -> None:
        """Contract defaults: all off, 20 degC, resident living_room, 08:00."""
        self.clock_hmin = float(clock_hmin)
        self.devices: Dict[str, Any] = {
            "thermostat.main": {"target": 20.0, "mode": "off"},
            "hvac.heater": "off",
            "lock.front_door": "unlocked",
            "door.front": "closed",
            "blinds.living": "open",
            "security.system": "disarmed",
        }
        for i in LIGHTS:
            self.devices[i] = {"on": False, "brightness": 70}
        for i in SWITCHES:
            self.devices[i] = "off"
        self.coffee_ready = False
        self._coffee_on_since: Optional[float] = None
        self.meta: Dict[str, Dict[str, Any]] = {
            i: {"last_change": None, "changed_by": None} for i in ALL_DEVICE_IDS
        }
        self.temps: Dict[str, float] = {r: 20.0 for r in ROOMS}
        self.energy_wh: Dict[str, float] = {i: 0.0 for i in ALL_DEVICE_IDS}
        self.energy_wh["standby"] = 0.0
        self.resident = "living_room"
        self.device_log: List[Dict[str, Any]] = []
        self.notifications: List[Dict[str, Any]] = []
        self.anomalies_active: List[Dict[str, Any]] = []
        self.anomalies_history: List[Dict[str, Any]] = []
        self._spike_started: Optional[float] = None
        self.timeline: List[Dict[str, Any]] = []
        self._next_sample_hmin = self.clock_hmin
        self.scenario: Optional[str] = None
        self.scenario_seed: Optional[int] = None
        self._scenario_t0_hmin = self.clock_hmin
        self._scenario_fired = 0
        self.outside_override: Optional[float] = None
        # Free-running is the interactive default; /scenario/start holds the
        # clock (hold_clock=true) so house time integrates ONLY inside an
        # explicit /scenario/advance window — see skip_to().
        self.clock_held = False
        # Push the default state into the scene.
        self._push_all_visuals()
        self._sample_timeline(force=True)

    def _push_all_visuals(self) -> None:
        for i in LIGHTS:
            st = self.devices[i]
            self.scene.apply_light(i, st["on"], st["brightness"])
        for i in SWITCHES:
            self.scene.apply_switch(i, self.devices[i] == "on")
        self.scene.apply_heater(self.devices["hvac.heater"] == "on")
        self.scene.apply_door(self.devices["door.front"] == "open")
        self.scene.apply_blinds(self.devices["blinds.living"] == "open")
        self.scene.apply_security(self.devices["security.system"] == "armed")
        self.scene.apply_resident(self.resident)

    # ── Clock ────────────────────────────────────────────────────────

    def house_time(self) -> str:
        return house_time_iso(self.clock_hmin)

    def skip_to(self, sim_now_s: float) -> None:
        """Move the sim-time anchor WITHOUT integrating.

        Called every tick while the clock is held outside an advance
        window: the engine keeps stepping, the house does not age, and
        the next advance() sees a fresh anchor instead of a time jump."""
        self._last_sim_t = sim_now_s

    def outside_temp(self) -> float:
        if self.outside_override is not None:
            return float(self.outside_override)
        return outside_temp_c(self.clock_hmin)

    # ── The one commit path for device state ─────────────────────────

    def _commit(self, dev_id: str, new_state: Any, changed_by: str,
                log_state: Optional[Any] = None) -> None:
        self.devices[dev_id] = new_state
        self.meta[dev_id] = {"last_change": self.house_time(),
                             "changed_by": changed_by}
        self.device_log.append({
            "house_time": self.house_time(),
            "id": dev_id,
            "state": log_state if log_state is not None else new_state,
            "changed_by": changed_by,
        })
        del self.device_log[:-LOG_MAX_ENTRIES]
        # Visual side effect
        t = DEVICE_TYPE[dev_id]
        if t == "light":
            self.scene.apply_light(dev_id, new_state["on"], new_state["brightness"])
        elif t == "switch":
            self.scene.apply_switch(dev_id, new_state == "on")
        elif dev_id == "hvac.heater":
            self.scene.apply_heater(new_state == "on")
        elif dev_id == "door.front":
            self.scene.apply_door(new_state == "open")
        elif dev_id == "blinds.living":
            self.scene.apply_blinds(new_state == "open")
        elif dev_id == "security.system":
            self.scene.apply_security(new_state == "armed")
        # thermostat.main / lock.front_door have no visual.
        # Track the coffee timer.
        if dev_id == "appliance.coffee_maker":
            if new_state == "on":
                self._coffee_on_since = self.clock_hmin
            else:
                self._coffee_on_since = None
                if self.coffee_ready:
                    self.coffee_ready = False

    # ── Device commands (validation + state; NO settle, NO measurement:
    #    the bridge measures after the engine has stepped) ─────────────

    def apply_set_device(self, dev_id: Any, state: Any,
                         changed_by: str) -> Optional[Dict[str, Any]]:
        """Validate and commit a set_device. Returns a fault dict on
        rejection, or None on success (bridge then settles + measures)."""
        if not isinstance(dev_id, str) or dev_id not in ALL_DEVICE_IDS:
            return fault("device_offline", f"unknown device id {dev_id!r}")
        if not self.scene.is_online(dev_id):
            return fault("device_offline",
                         f"{dev_id} did not resolve in the scene at startup")
        t = DEVICE_TYPE[dev_id]
        if t == "light":
            cur = dict(self.devices[dev_id])
            if state == "on":
                cur["on"] = True
            elif state == "off":
                cur["on"] = False
            elif isinstance(state, dict):
                if "on" in state:
                    if not isinstance(state["on"], bool):
                        return fault("state_rejected", "'on' must be a bool")
                    cur["on"] = state["on"]
                if "brightness" in state:
                    b = state["brightness"]
                    if isinstance(b, bool) or not isinstance(b, (int, float)):
                        return fault("state_rejected", "'brightness' must be 0-100")
                    cur["brightness"] = max(0, min(100, float(b)))
                    if "on" not in state and cur["brightness"] > 0:
                        cur["on"] = True
            else:
                return fault("state_rejected",
                             "light state must be 'on', 'off' or "
                             "{'on': bool, 'brightness': 0-100}")
            self._commit(dev_id, cur, changed_by)
            return None
        if t == "switch":
            if state not in ("on", "off"):
                return fault("state_rejected", "switch state must be 'on' or 'off'")
            self._commit(dev_id, state, changed_by)
            return None
        if dev_id == "thermostat.main":
            if not isinstance(state, dict):
                return fault("state_rejected",
                             "thermostat state is {'target': degC, 'mode': "
                             "'heat'|'eco'|'off'}")
            return self.apply_adjust_thermostat(state.get("target"),
                                                state.get("mode"), changed_by)[0]
        if dev_id == "hvac.heater":
            return fault("state_rejected",
                         "hvac.heater is internal: it follows the thermostat "
                         "(adjust_thermostat), it cannot be set directly")
        if dev_id == "lock.front_door":
            if state == "locked":
                self._commit(dev_id, "locked", changed_by)
                return None
            if state == "unlocked":
                return fault("authorization_required",
                             "unlocking needs unlock_door with authorization")
            return fault("state_rejected", "lock state must be 'locked' or 'unlocked'")
        if dev_id == "door.front":
            if state not in ("open", "closed"):
                return fault("state_rejected", "door state must be 'open' or 'closed'")
            self._commit(dev_id, state, changed_by)
            return None
        if dev_id == "blinds.living":
            if state not in ("open", "closed"):
                return fault("state_rejected", "blinds state must be 'open' or 'closed'")
            self._commit(dev_id, state, changed_by)
            return None
        if dev_id == "security.system":
            if state == "armed":
                self._commit(dev_id, "armed", changed_by)
                return None
            if state == "disarmed":
                return fault("authorization_required",
                             "disarming needs disarm_security with authorization")
            return fault("state_rejected", "alarm state must be 'armed' or 'disarmed'")
        return fault("state_rejected", f"cannot set {dev_id}")  # pragma: no cover

    def apply_toggle_device(self, dev_id: Any,
                            changed_by: str) -> Optional[Dict[str, Any]]:
        if not isinstance(dev_id, str) or dev_id not in ALL_DEVICE_IDS:
            return fault("device_offline", f"unknown device id {dev_id!r}")
        t = DEVICE_TYPE[dev_id]
        if t == "light":
            return self.apply_set_device(
                dev_id, "off" if self.devices[dev_id]["on"] else "on", changed_by)
        if t == "switch":
            return self.apply_set_device(
                dev_id, "off" if self.devices[dev_id] == "on" else "on", changed_by)
        if dev_id in ("door.front", "blinds.living"):
            return self.apply_set_device(
                dev_id, "closed" if self.devices[dev_id] == "open" else "open",
                changed_by)
        return fault("state_rejected", f"{dev_id} is not toggleable")

    def apply_adjust_thermostat(self, target: Any, mode: Any, changed_by: str
                                ) -> Tuple[Optional[Dict[str, Any]], bool]:
        """Returns (fault_or_None, clamped)."""
        cur = dict(self.devices["thermostat.main"])
        clamped = False
        if target is not None:
            if isinstance(target, bool) or not isinstance(target, (int, float)):
                return fault("state_rejected", "'target' must be a number"), False
            t = float(target)
            if t < 5.0 or t > 30.0:
                t = max(5.0, min(30.0, t))
                clamped = True
            cur["target"] = t
        if mode is not None:
            if mode not in ("heat", "eco", "off"):
                return fault("state_rejected",
                             "'mode' must be 'heat', 'eco' or 'off'"), False
            cur["mode"] = mode
        if target is None and mode is None:
            return fault("state_rejected", "nothing to adjust: pass target and/or mode"), False
        self._commit("thermostat.main", cur, changed_by)
        return None, clamped

    def apply_scene(self, scene_name: Any, changed_by: str
                    ) -> Tuple[Optional[Dict[str, Any]], List[str]]:
        """Returns (fault_or_None, affected_device_ids)."""
        if not isinstance(scene_name, str) or scene_name not in SCENES:
            return fault("scene_unknown",
                         f"unknown scene {scene_name!r}; scenes: {sorted(SCENES)}"), []
        affected: List[str] = []
        for dev_id, state in SCENES[scene_name]:
            if dev_id == "thermostat.main":
                self.apply_adjust_thermostat(state.get("target"),
                                             state.get("mode"), changed_by)
            elif dev_id == "lock.front_door":
                self._commit(dev_id, state, changed_by)
            elif dev_id == "security.system":
                self._commit(dev_id, state, changed_by)
            else:
                err = self.apply_set_device(dev_id, state, changed_by)
                if err is not None:  # offline device: skip, still report the rest
                    continue
            affected.append(dev_id)
        return None, affected

    # ── Scenario engine ──────────────────────────────────────────────

    def scenario_start(self, name: Any, seed: Any = None,
                       time_scale: Any = None,
                       start_time: Any = None,
                       hold_clock: Any = True) -> Optional[Dict[str, Any]]:
        if not isinstance(name, str) or name not in SCENARIOS:
            return fault("state_rejected",
                         f"unknown scenario {name!r}; known: {sorted(SCENARIOS)}")
        spec = SCENARIOS[name]
        if time_scale is not None:
            try:
                ts = float(time_scale)
            except (TypeError, ValueError):
                return fault("state_rejected", "time_scale must be a number")
            if not (1.0 <= ts <= 3600.0):
                return fault("state_rejected", "time_scale must be in [1, 3600]")
            self.time_scale = ts
        clock = None
        if start_time is not None:
            clock = parse_start_time(start_time)
            if clock is None:
                return fault("state_rejected",
                             "start_time must be 'HH:MM' or an ISO datetime")
        if clock is None:
            h, m = spec["start_hm"]
            clock = h * 60.0 + m
        # reset() also clears metrics / notifications / device_log for the
        # new scenario (contract update #1).
        self.reset(clock_hmin=clock)
        self.scenario = name
        self.scenario_seed = seed if isinstance(seed, int) else None
        self._scenario_t0_hmin = self.clock_hmin
        self._scenario_fired = 0
        # Benchmark mode by default: hold the clock so house time moves
        # only inside /scenario/advance windows. hold_clock=false keeps
        # the interactive free-running behaviour.
        self.clock_held = bool(hold_clock) if hold_clock is not None else True
        self._set_resident(spec["resident"], "scenario")
        return None

    def scenario_status(self) -> Dict[str, Any]:
        spec = SCENARIOS.get(self.scenario or "", {})
        total = len(spec.get("timeline", []))
        if self.scenario is None:
            phase = "idle"
        elif self._scenario_fired >= total:
            phase = "complete"
        else:
            phase = "running"
        return {
            "name": self.scenario,
            "house_time": self.house_time(),
            "phase": phase,
            "resident": self.resident,
            "elapsed_house_min": round(self.clock_hmin - self._scenario_t0_hmin, 3),
            "clock": "held" if self.clock_held else "free_running",
        }

    def scenario_event(self, ev: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Apply one scripted actor event (changed_by='scenario')."""
        ev_type = ev.get("type")
        if ev_type == "resident":
            room = ev.get("room")
            if room not in ROOMS and room != "away":
                return fault("room_unknown", f"unknown room {room!r}")
            self._set_resident(room, "scenario")
            return None
        if ev_type in ("device_set", "door"):
            return self.apply_set_device(ev.get("id"), ev.get("state"), "scenario")
        return fault("state_rejected",
                     "event type must be 'device_set', 'door' or 'resident'")

    def _set_resident(self, room_or_away: str, changed_by: str) -> None:
        self.resident = room_or_away
        self.scene.apply_resident(room_or_away)
        self.device_log.append({
            "house_time": self.house_time(),
            "id": "resident",
            "state": room_or_away,
            "changed_by": changed_by,
        })

    def _fire_due_scenario_events(self) -> None:
        if self.scenario is None:
            return
        timeline = SCENARIOS[self.scenario]["timeline"]
        elapsed = self.clock_hmin - self._scenario_t0_hmin
        while self._scenario_fired < len(timeline):
            offset, ev = timeline[self._scenario_fired]
            if elapsed < offset:
                break
            self.scenario_event(ev)
            self._scenario_fired += 1

    # ── Power / energy ───────────────────────────────────────────────

    def device_watts(self, dev_id: str) -> float:
        if dev_id in LIGHTS:
            st = self.devices[dev_id]
            return LIGHTS[dev_id]["watts"] * (st["brightness"] / 100.0) if st["on"] else 0.0
        if dev_id in SWITCHES:
            return SWITCHES[dev_id]["watts"] if self.devices[dev_id] == "on" else 0.0
        if dev_id == "hvac.heater":
            return HEATER_WATTS if self.devices[dev_id] == "on" else 0.0
        return 0.0

    def total_watts(self) -> float:
        return sum(self.device_watts(i) for i in ALL_DEVICE_IDS) + STANDBY_WATTS

    def energy_by_category(self, energy: Optional[Dict[str, float]] = None
                           ) -> Dict[str, float]:
        e = energy if energy is not None else self.energy_wh
        return {
            "hvac": e.get("hvac.heater", 0.0),
            "lighting": sum(e.get(i, 0.0) for i in LIGHTS),
            "appliances": sum(e.get(i, 0.0) for i in SWITCHES),
            "standby": e.get("standby", 0.0),
        }

    # ── The per-step advance ─────────────────────────────────────────

    def advance(self, sim_now_s: float) -> None:
        """Integrate the house forward to simulator time ``sim_now_s``."""
        if self._last_sim_t is None:
            self._last_sim_t = sim_now_s
            return
        dt_s = sim_now_s - self._last_sim_t
        self._last_sim_t = sim_now_s
        if dt_s <= 0:
            return
        dt_hmin = dt_s * self.time_scale / 60.0
        self.clock_hmin += dt_hmin

        # Scenario timeline
        self._fire_due_scenario_events()

        # Thermostat hysteresis automation (sensor = hallway; changed_by=hub)
        th = self.devices["thermostat.main"]
        heater_on = self.devices["hvac.heater"] == "on"
        if th["mode"] == "off":
            if heater_on:
                self._commit("hvac.heater", "off", "hub")
        else:
            eff = th["target"] - (ECO_SETBACK_C if th["mode"] == "eco" else 0.0)
            t_sense = self.temps["hallway"]
            if not heater_on and t_sense < eff - HYSTERESIS_C:
                self._commit("hvac.heater", "on", "hub")
            elif heater_on and t_sense > eff + HYSTERESIS_C:
                self._commit("hvac.heater", "off", "hub")

        # Temperatures
        t_out = self.outside_temp()
        oven_on = self.devices["appliance.oven"] == "on"
        heater_on = self.devices["hvac.heater"] == "on"
        for room in ROOMS:
            rate = (t_out - self.temps[room]) / TAU_HMIN
            if heater_on:
                rate += RATE_HEATER
            if oven_on:
                if room == "kitchen":
                    rate += RATE_OVEN_KITCHEN
                elif room == "hallway":
                    rate += RATE_OVEN_HALLWAY
            self.temps[room] += rate * dt_hmin

        # Energy
        dt_hh = dt_hmin / 60.0
        for i in ALL_DEVICE_IDS:
            w = self.device_watts(i)
            if w:
                self.energy_wh[i] += w * dt_hh
        self.energy_wh["standby"] += STANDBY_WATTS * dt_hh

        # Coffee readiness
        if (self._coffee_on_since is not None and not self.coffee_ready
                and self.clock_hmin - self._coffee_on_since >= COFFEE_READY_HMIN):
            self.coffee_ready = True
            self.device_log.append({
                "house_time": self.house_time(),
                "id": "appliance.coffee_maker",
                "state": "coffee_ready",
                "changed_by": "hub",
            })

        # Anomalies
        self._update_anomalies()

        # Timeline sampling
        if self.clock_hmin >= self._next_sample_hmin:
            self._sample_timeline()

    def _update_anomalies(self) -> None:
        # door_open_while_armed
        cond = (self.devices["door.front"] == "open"
                and self.devices["security.system"] == "armed")
        active = next((a for a in self.anomalies_active
                       if a["type"] == "door_open_while_armed"), None)
        if cond and active is None:
            entry = {"type": "door_open_while_armed", "id": "door.front",
                     "started_house_time": self.house_time(),
                     "ended_house_time": None}
            self.anomalies_active.append(entry)
            self.anomalies_history.append(entry)
        elif not cond and active is not None:
            active["ended_house_time"] = self.house_time()
            self.anomalies_active.remove(active)

        # energy_spike: total draw above threshold, sustained, occupant away
        watts = self.total_watts()
        spike_active = next((a for a in self.anomalies_active
                             if a["type"] == "energy_spike"), None)
        if watts > ENERGY_SPIKE_WATTS and self.resident == "away":
            if self._spike_started is None:
                self._spike_started = self.clock_hmin
            sustained = self.clock_hmin - self._spike_started
            if sustained >= ENERGY_SPIKE_SUSTAIN_HMIN and spike_active is None:
                entry = {"type": "energy_spike", "id": None,
                         "watts": round(watts, 1),
                         "started_house_time": house_time_iso(self._spike_started),
                         "ended_house_time": None}
                self.anomalies_active.append(entry)
                self.anomalies_history.append(entry)
            elif spike_active is not None:
                spike_active["watts"] = round(max(spike_active["watts"], watts), 1)
        else:
            self._spike_started = None
            if spike_active is not None:
                spike_active["ended_house_time"] = self.house_time()
                self.anomalies_active.remove(spike_active)

    def _sample_timeline(self, force: bool = False) -> None:
        self.timeline.append({
            "hmin": round(self.clock_hmin, 3),
            "house_time": self.house_time(),
            "temps": {r: round(t, 3) for r, t in self.temps.items()},
            "outside": round(self.outside_temp(), 3),
            "total_wh": round(sum(self.energy_wh.values()), 3),
            "by_category": {k: round(v, 3)
                            for k, v in self.energy_by_category().items()},
            "by_device": {i: round(v, 3) for i, v in self.energy_wh.items() if v},
        })
        del self.timeline[:-TIMELINE_MAX_SAMPLES]
        self._next_sample_hmin = self.clock_hmin + TIMELINE_SAMPLE_HMIN

    # ── Reports ──────────────────────────────────────────────────────

    def energy_report(self, range_spec: Any) -> Dict[str, Any]:
        """range: 'all' (default) or '<N>h' / {'hours': N} — house time."""
        hours: Optional[float] = None
        if isinstance(range_spec, dict):
            try:
                hours = float(range_spec.get("hours"))
            except (TypeError, ValueError):
                hours = None
        elif isinstance(range_spec, str) and range_spec.strip().lower().endswith("h"):
            try:
                hours = float(range_spec.strip().lower()[:-1])
            except ValueError:
                hours = None
        now_total = sum(self.energy_wh.values())
        base_by_dev: Dict[str, float] = {}
        window_start_hmin = self.timeline[0]["hmin"] if self.timeline else self.clock_hmin
        if hours is not None and self.timeline:
            cutoff = self.clock_hmin - hours * 60.0
            base = None
            for s in self.timeline:
                if s["hmin"] >= cutoff:
                    base = s
                    break
            if base is not None:
                base_by_dev = dict(base.get("by_device", {}))
                window_start_hmin = base["hmin"]
        by_dev_delta = {i: self.energy_wh[i] - base_by_dev.get(i, 0.0)
                        for i in self.energy_wh}
        total_delta = max(0.0, now_total - sum(base_by_dev.values()))
        by_cat = self.energy_by_category(by_dev_delta)
        outliers = []
        for i, wh in sorted(by_dev_delta.items(), key=lambda kv: -kv[1]):
            if i == "standby":
                continue
            if total_delta > 0 and wh > 100.0 and wh / total_delta > 0.35:
                outliers.append({"id": i, "wh": round(wh, 1),
                                 "share": round(wh / total_delta, 3)})
        return {
            "total_kwh": round(total_delta / 1000.0, 4),
            "by_category": {k: round(v / 1000.0, 4) for k, v in by_cat.items()},
            "outliers": outliers,
            "window_house_min": round(self.clock_hmin - window_start_hmin, 1),
            "house_time": self.house_time(),
        }

    def read_sensors(self, room: Any = None) -> Any:
        if room is not None and room not in ROOMS and room != "outside":
            return fault("room_unknown",
                         f"unknown room {room!r}; rooms: {ROOMS + ['outside']}")
        ts = self.house_time()
        # The measured resident room, from the prop's real position when the
        # scene can provide it (never the commanded value).
        pos = self.scene.read_resident_position()
        measured_room = (room_from_position(*pos) if pos is not None
                         else ("away" if self.resident == "away" else None))
        readings: List[Dict[str, Any]] = []
        for r in ROOMS:
            if room is not None and r != room:
                continue
            readings.append({"room_id": r, "type": "temperature",
                             "value": round(self.temps[r], 2), "unit": "C",
                             "timestamp": ts})
            readings.append({"room_id": r, "type": "motion",
                             "value": (measured_room == r
                                       if measured_room is not None else None),
                             "timestamp": ts})
        if room is None or room == "outside":
            readings.append({"room_id": "outside", "type": "temperature",
                             "value": round(self.outside_temp(), 2), "unit": "C",
                             "timestamp": ts})
        return {"readings": readings}

    def notify(self, message: str, severity: Any, channel: Any,
               occupant_id: Any) -> Dict[str, Any]:
        entry = {
            "house_time": self.house_time(),
            "message": message,
            "severity": severity if severity in ("info", "warning", "critical")
                        else "info",
            "channel": "in_app_sim",
            "occupant_id": occupant_id,
        }
        self.notifications.append(entry)
        return {"delivered": True, "channel": "in_app_sim",
                "house_time": entry["house_time"]}

    def metrics(self) -> Dict[str, Any]:
        return {
            "energy_wh_total": round(sum(self.energy_wh.values()), 2),
            "energy_wh_by_device": {i: round(v, 2)
                                    for i, v in self.energy_wh.items() if v},
            "energy_wh_by_category": {k: round(v, 2)
                                      for k, v in self.energy_by_category().items()},
            "room_temps": {r: round(t, 2) for r, t in self.temps.items()},
            "outside_temp": round(self.outside_temp(), 2),
            "temp_timeline": [{"house_time": s["house_time"], "temps": s["temps"]}
                              for s in self.timeline],
            "notifications": list(self.notifications),
            "device_log": list(self.device_log),
            "anomalies_history": list(self.anomalies_history),
            "anomalies_active": list(self.anomalies_active),
            "resident": self.resident,
            "house_time": self.house_time(),
        }
