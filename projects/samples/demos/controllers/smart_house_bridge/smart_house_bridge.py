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

"""smart_house_bridge — the OmniLink Smart House hub.

Supervisor controller for
``projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld``.
It serves the Haven smart-home hub HTTP contract on ``127.0.0.1:8766``
(``--port`` overrides): 19 hub verbs (lights / appliances / thermostat /
lock / blinds / security / sensors / energy / anomalies / weather /
notify), a POST-only ``/scenario/*`` engine for the benchmark, and the
PROTOCOL.md robot-bridge conformance surface (``GET /protocol``,
``/capabilities``, ``/state``, ``POST /action``, ``/stop_robot``, ...).

Design rules baked in (PROTOCOL.md §5.4.1 + the frozen demo contract):

- **Measured, never echoed.** Supervisor field writes land on the
  engine's NEXT step, so every mutating verb queues its write to the
  main step loop, waits one settle step, reads the scene back, and
  answers with the measured state. Devices with no physical carrier
  (thermostat setpoint, lock, alarm) answer from the hub registry —
  the hub IS their ground truth — and that is documented, not hidden.
- **Command rejections are HTTP 200** with
  ``{"accepted": false, "error": "<fault>", "message": ...}`` using the
  Haven fault codes (device_offline / state_rejected /
  authorization_required / degraded / scene_unknown / room_unknown).
  Transport-level 4xx only for malformed requests; unknown endpoints
  are 404 ``{"ok": false, "error": "unknown_endpoint"}``.
- **One thread owns the Supervisor.** HTTP handler threads never touch
  libController; they submit jobs to the main step loop and wait.
- ``/scenario/advance`` blocks until the engine has actually stepped
  the equivalent sim time (max 480 house-minutes per call — chunk
  longer advances client-side).

Stdlib only. The wire-safety helpers come from the shared
``_omnilink_relay.http_security`` sibling module (itself stdlib-only).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional, Tuple

from omnisim import Supervisor

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
for _p in (_THIS_DIR, _PARENT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from _omnilink_relay.http_security import (  # noqa: E402
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    read_json,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

import house_model as hm  # noqa: E402
from house_model import HouseModel, SceneInterface, fault  # noqa: E402

TAG = "[smart_house_bridge]"

MAX_ADVANCE_HOUSE_MIN = 480.0

HUB_VERBS = [
    "list_rooms", "list_devices", "read_sensors", "get_device_state",
    "set_device", "toggle_device", "set_scene", "adjust_thermostat",
    "set_schedule", "lock_door", "unlock_door", "arm_security",
    "disarm_security", "get_energy_report", "check_anomalies",
    "get_weather", "notify_occupant", "shut_water_main", "shut_gas_main",
]
# Paths whose JSON payload uses 'id' as a DEVICE id (Haven adapters.md),
# so the bare 'id' field must never be consumed as an idempotency key.
DEVICE_ID_PATHS = {
    "/set_device", "/toggle_device", "/get_device_state",
    "/adjust_thermostat", "/lock_door", "/unlock_door", "/set_schedule",
    "/scenario/event", "/action",
}
READONLY_PATHS = {
    "/list_rooms", "/list_devices", "/read_sensors", "/get_device_state",
    "/get_energy_report", "/check_anomalies", "/get_weather",
    "/scenario/status", "/scenario/metrics", "/state", "/get_robot_state",
    "/list_robots", "/capabilities", "/healthz",
}

CAPABILITIES_BY_TYPE = {
    "light": ["on_off", "brightness"],
    "switch": ["on_off"],
    "thermostat": ["target", "mode"],
    "lock": ["lock", "unlock_with_authorization"],
    "door": ["open_close"],
    "cover": ["open_close"],
    "alarm": ["arm", "disarm_with_authorization"],
    "internal": ["read_only"],
}

MISSION_BRIEF = (
    "You are the hub of a one-floor smart house (living_room, kitchen, "
    "bedroom, hallway; front door + windows form the perimeter). Devices: "
    "6 lights, oven, coffee maker, TV, a single-zone thermostat driving a "
    "heater, front-door lock, the physical front door, living-room blinds "
    "and a security system. Sensors: per-room temperature and motion, plus "
    "outside temperature. Keep the occupant safe and comfortable and keep "
    "energy use sensible. All state you read is measured from the "
    "simulation; commands land on the next physics step."
)


# ── Scene bindings (the only code that touches the Supervisor) ────────

# device id -> (PointLight DEF, fixture material DEF, max intensity)
LIGHT_BINDINGS = {
    "lights.living_ceiling": ("PL_LIVING_CEILING", "MAT_LIVING_CEILING", 8.0),
    "lights.living_lamp": ("PL_LIVING_LAMP", "MAT_LIVING_LAMP", 4.0),
    "lights.kitchen_ceiling": ("PL_KITCHEN_CEILING", "MAT_KITCHEN_CEILING", 8.0),
    "lights.bedroom_ceiling": ("PL_BEDROOM_CEILING", "MAT_BEDROOM_CEILING", 8.0),
    "lights.bedroom_lamp": ("PL_BEDROOM_LAMP", "MAT_BEDROOM_LAMP", 4.0),
    "lights.hallway": ("PL_HALLWAY", "MAT_HALLWAY", 8.0),
}
# device id -> (indicator material DEF, emissive colour when on)
INDICATOR_BINDINGS = {
    "appliance.oven": ("MAT_OVEN_IND", (1.0, 0.25, 0.05)),
    "appliance.coffee_maker": ("MAT_COFFEE_IND", (1.0, 0.6, 0.1)),
    "appliance.tv": ("MAT_TV_SCREEN", (0.35, 0.42, 0.55)),
    "hvac.heater": ("MAT_HEATER_IND", (1.0, 0.3, 0.05)),
}
DOOR_OPEN_RAD = 1.45
BLINDS_XY = (-2.5, 3.86)
BLINDS_CLOSED_Z = 0.0
BLINDS_OPEN_Z = -4.0
RESIDENT_ANCHOR = {
    "living_room": (-3.0, 2.5, 0.0),
    "kitchen": (2.5, 2.7, 0.0),
    "bedroom": (3.0, -1.5, 0.0),
    "hallway": (-2.5, -2.5, 0.0),
    "away": (0.0, 0.0, -10.0),
}
HUB_SCREEN_ARMED = (0.55, 0.03, 0.03)
HUB_SCREEN_DISARMED = (0.0, 0.06, 0.08)
FIXTURE_WARM = (0.85, 0.75, 0.5)


class SupervisorScene(SceneInterface):
    """SceneInterface on the live Supervisor.

    Resolves every DEF once at startup; a missing DEF marks its device
    offline (honest degradation) instead of crashing the hub. Emissive
    and intensity writes are gated behind a >1% change — every
    PointLight-intensity write triggers an IBL rebake.
    """

    def __init__(self, sup: Supervisor):
        self.sup = sup
        self.missing: List[str] = []
        self._offline: set = set()

        def field(def_name: str, field_name: str):
            node = sup.getFromDef(def_name)
            if node is None:
                self.missing.append(def_name)
                return None
            f = node.getField(field_name)
            if f is None:
                self.missing.append(f"{def_name}.{field_name}")
            return f

        self.lights: Dict[str, Dict[str, Any]] = {}
        for dev, (pl_def, mat_def, max_int) in LIGHT_BINDINGS.items():
            b = {
                "on": field(pl_def, "on"),
                "intensity": field(pl_def, "intensity"),
                "emissive": field(mat_def, "emissiveColor"),
                "max": max_int,
                "last_intensity": None,
                "last_on": None,
                "last_emissive": None,
            }
            if b["on"] is None or b["intensity"] is None:
                self._offline.add(dev)
            self.lights[dev] = b

        self.indicators: Dict[str, Dict[str, Any]] = {}
        for dev, (mat_def, color) in INDICATOR_BINDINGS.items():
            f = field(mat_def, "emissiveColor")
            if f is None:
                self._offline.add(dev)
            self.indicators[dev] = {"emissive": f, "color": color, "last": None}

        self.door_rot = field("FRONT_DOOR", "rotation")
        if self.door_rot is None:
            self._offline.add("door.front")
        self.blinds_tr = field("BLINDS_LIVING", "translation")
        if self.blinds_tr is None:
            self._offline.add("blinds.living")
        self.resident_tr = field("RESIDENT", "translation")
        self.hub_screen = field("MAT_HUB_SCREEN", "emissiveColor")

        if self.missing:
            print(f"{TAG} WARNING: unresolved scene bindings (devices "
                  f"degraded to offline): {sorted(set(self.missing))}",
                  flush=True)

    # -- helpers ------------------------------------------------------

    @staticmethod
    def _set_color_gated(binding_key: str, f, cache: Dict[str, Any],
                         rgb: Tuple[float, float, float]) -> None:
        last = cache.get(binding_key)
        if f is None:
            return
        if last is not None and all(abs(a - b) <= 0.01 for a, b in zip(last, rgb)):
            return
        f.setSFColor(list(rgb))
        cache[binding_key] = tuple(rgb)

    # -- SceneInterface: apply ----------------------------------------

    def apply_light(self, dev_id: str, on: bool, brightness: float) -> None:
        b = self.lights.get(dev_id)
        if not b or b["on"] is None:
            return
        want_on = bool(on) and brightness > 0
        intensity = b["max"] * (brightness / 100.0) if want_on else 0.0
        if b["last_intensity"] is None or \
                abs(intensity - b["last_intensity"]) > 0.01 * b["max"]:
            b["intensity"].setSFFloat(intensity)
            b["last_intensity"] = intensity
        if b["last_on"] is None or b["last_on"] != want_on:
            b["on"].setSFBool(want_on)
            b["last_on"] = want_on
        if b["emissive"] is not None:
            scale = (0.25 + 0.75 * brightness / 100.0) if want_on else 0.0
            self._set_color_gated("last_emissive", b["emissive"], b,
                                  tuple(c * scale for c in FIXTURE_WARM))

    def apply_switch(self, dev_id: str, on: bool) -> None:
        ind = self.indicators.get(dev_id)
        if not ind or ind["emissive"] is None:
            return
        rgb = ind["color"] if on else (0.0, 0.0, 0.0)
        self._set_color_gated("last", ind["emissive"], ind, rgb)

    def apply_heater(self, on: bool) -> None:
        self.apply_switch("hvac.heater", on)

    def apply_door(self, is_open: bool) -> None:
        if self.door_rot is not None:
            self.door_rot.setSFRotation([0.0, 0.0, 1.0,
                                         DOOR_OPEN_RAD if is_open else 0.0])

    def apply_blinds(self, is_open: bool) -> None:
        if self.blinds_tr is not None:
            z = BLINDS_OPEN_Z if is_open else BLINDS_CLOSED_Z
            self.blinds_tr.setSFVec3f([BLINDS_XY[0], BLINDS_XY[1], z])

    def apply_security(self, armed: bool) -> None:
        if self.hub_screen is not None:
            self.hub_screen.setSFColor(
                list(HUB_SCREEN_ARMED if armed else HUB_SCREEN_DISARMED))

    def apply_resident(self, room_or_away: str) -> None:
        if self.resident_tr is not None:
            anchor = RESIDENT_ANCHOR.get(room_or_away, RESIDENT_ANCHOR["away"])
            self.resident_tr.setSFVec3f(list(anchor))

    # -- SceneInterface: read (measurements) --------------------------

    def read_light(self, dev_id: str) -> Optional[Dict[str, Any]]:
        b = self.lights.get(dev_id)
        if not b or b["on"] is None or b["intensity"] is None:
            return None
        on = bool(b["on"].getSFBool())
        intensity = float(b["intensity"].getSFFloat())
        brightness = round(min(100.0, 100.0 * intensity / b["max"]), 1) if on else 0.0
        return {"on": on, "brightness": brightness}

    def read_switch(self, dev_id: str) -> Optional[bool]:
        ind = self.indicators.get(dev_id)
        if not ind or ind["emissive"] is None:
            return None
        rgb = ind["emissive"].getSFColor()
        return max(rgb) > 0.03

    def read_heater(self) -> Optional[bool]:
        return self.read_switch("hvac.heater")

    def read_door_open(self) -> Optional[bool]:
        if self.door_rot is None:
            return None
        rot = self.door_rot.getSFRotation()
        return abs(float(rot[3])) > 0.7

    def read_blinds_open(self) -> Optional[bool]:
        if self.blinds_tr is None:
            return None
        return float(self.blinds_tr.getSFVec3f()[2]) < -1.0

    def read_resident_position(self) -> Optional[Tuple[float, float, float]]:
        if self.resident_tr is None:
            return None
        v = self.resident_tr.getSFVec3f()
        return (float(v[0]), float(v[1]), float(v[2]))

    def is_online(self, dev_id: str) -> bool:
        return dev_id not in self._offline


# ── Main-loop executor: HTTP threads -> supervisor thread ─────────────

class MainLoopExecutor:
    """Runs jobs on the simulation thread.

    ``submit(apply_fn)``: apply_fn runs at the next step boundary. If it
    returns a dict, that is the final result (fault or a response that
    needs no settle). If it returns None and a ``read_fn`` was given,
    the job waits ONE engine step and then read_fn() produces the
    result — that is how "realized_state is measured after >=1 settle
    step" is implemented.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._pending: List[Dict[str, Any]] = []
        self._settling: List[Dict[str, Any]] = []

    def submit(self, apply_fn: Callable[[], Optional[Dict[str, Any]]],
               read_fn: Optional[Callable[[], Dict[str, Any]]] = None,
               timeout_s: float = 30.0) -> Dict[str, Any]:
        job = {"apply": apply_fn, "read": read_fn,
               "done": threading.Event(), "result": None, "exc": None}
        with self._lock:
            self._pending.append(job)
        if not job["done"].wait(timeout_s):
            return fault("degraded",
                         "the simulation loop is not stepping; try again")
        if job["exc"] is not None:
            raise job["exc"]
        return job["result"]

    def run_cycle(self) -> None:
        """Called once per main-loop iteration, right after robot.step()."""
        with self._lock:
            settling, self._settling = self._settling, []
            pending, self._pending = self._pending, []
        for job in settling:
            try:
                job["result"] = job["read"]()
            except Exception as e:  # noqa: BLE001
                job["exc"] = e
            job["done"].set()
        hold: List[Dict[str, Any]] = []
        for job in pending:
            try:
                r = job["apply"]()
            except Exception as e:  # noqa: BLE001
                job["exc"] = e
                job["done"].set()
                continue
            if r is None and job["read"] is not None:
                hold.append(job)  # settle one step, read next cycle
            else:
                job["result"] = r
                job["done"].set()
        if hold:
            with self._lock:
                self._settling.extend(hold)


# ── The bridge ────────────────────────────────────────────────────────

class SmartHouseBridge:
    def __init__(self, sup: Supervisor, robot_id: str, port: int):
        self.sup = sup
        self.robot_id = robot_id
        self.port = port
        self.timestep = int(sup.getBasicTimeStep())
        self.scene = SupervisorScene(sup)
        self.model = HouseModel(self.scene)
        self.executor = MainLoopExecutor()
        self.sim_now = 0.0          # cached each loop; read by HTTP threads
        self.house_time_cache = self.model.house_time()
        self._adv_lock = threading.Lock()
        self._advance_waiters: List[Tuple[float, threading.Event]] = []
        # Open /scenario/advance windows. While the model's clock is HELD
        # (benchmark mode), house time integrates only when this is > 0.
        self._adv_open = 0
        try:
            self.world_path = sup.getWorldPath()
        except Exception:  # noqa: BLE001
            self.world_path = None

    # -- main loop ----------------------------------------------------

    def tick(self) -> None:
        self.sim_now = self.sup.getTime()
        # Two clock modes (README "Clock modes"): free-running (interactive
        # default — the house lives with the engine) vs held (a scenario
        # with hold_clock: house time integrates ONLY inside an explicit
        # /scenario/advance window; between windows the anchor is kept
        # fresh so no time jump happens when the next window opens).
        with self._adv_lock:
            window_open = self._adv_open > 0
        if self.model.clock_held and not window_open:
            self.model.skip_to(self.sim_now)
        else:
            self.model.advance(self.sim_now)
        self.house_time_cache = self.model.house_time()
        self.executor.run_cycle()
        with self._adv_lock:
            still = []
            for target, ev in self._advance_waiters:
                if self.sim_now >= target:
                    ev.set()
                    self._adv_open = max(0, self._adv_open - 1)
                else:
                    still.append((target, ev))
            self._advance_waiters = still

    def open_advance(self, house_minutes: float,
                     ev: threading.Event) -> Dict[str, Any]:
        """Register an advance waiter AND open the integration window in
        one main-thread step, so a held clock cannot leak time between
        the snapshot and the registration."""
        target_sim = self.sim_now + house_minutes * 60.0 / self.model.time_scale
        with self._adv_lock:
            self._advance_waiters.append((target_sim, ev))
            self._adv_open += 1
        return {"hmin": self.model.clock_hmin, "sim": self.sim_now,
                "target_sim": target_sim}

    def cancel_advance(self, ev: threading.Event) -> None:
        """Timeout path: drop the waiter and close its window."""
        with self._adv_lock:
            before = len(self._advance_waiters)
            self._advance_waiters = [(t, e) for t, e in self._advance_waiters
                                     if e is not ev]
            if len(self._advance_waiters) != before:
                self._adv_open = max(0, self._adv_open - 1)

    # -- measurement --------------------------------------------------

    def measure_device(self, dev_id: str) -> Any:
        """The measured state of one device (scene first, hub registry
        for devices with no physical carrier)."""
        model = self.model
        t = hm.DEVICE_TYPE[dev_id]
        if t == "light":
            m = self.scene.read_light(dev_id)
            if m is not None:
                return m
            return None  # offline: never invent a measurement
        if t == "switch":
            m = self.scene.read_switch(dev_id)
            return None if m is None else ("on" if m else "off")
        if dev_id == "door.front":
            m = self.scene.read_door_open()
            return None if m is None else ("open" if m else "closed")
        if dev_id == "blinds.living":
            m = self.scene.read_blinds_open()
            return None if m is None else ("open" if m else "closed")
        if dev_id == "hvac.heater":
            m = self.scene.read_heater()
            return None if m is None else ("on" if m else "off")
        # thermostat.main / lock.front_door / security.system: the hub
        # registry IS the ground truth (no physical carrier to measure).
        st = model.devices[dev_id]
        return dict(st) if isinstance(st, dict) else st

    def transact(self, apply_fn, dev_ids_for_read=None, extra=None):
        """Apply a mutation, settle >=1 engine step, answer measured."""
        extra = extra or {}

        def read():
            if callable(dev_ids_for_read):
                out = dev_ids_for_read()
            elif isinstance(dev_ids_for_read, str):
                out = {"realized_state": self.measure_device(dev_ids_for_read)}
            else:
                out = {}
            resp = {"accepted": True}
            resp.update(extra)
            resp.update(out)
            return resp

        return self.executor.submit(apply_fn, read)


# ── HTTP layer ────────────────────────────────────────────────────────

def _finite(obj):
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    if isinstance(obj, dict):
        return {k: _finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite(v) for v in obj]
    return obj


def make_handler(bridge: SmartHouseBridge):
    trusted_origins = allowed_origins()
    bridge_token = configured_token()
    request_ids = RequestIdGuard()
    model = bridge.model

    def capabilities_body() -> Dict[str, Any]:
        sensors = (
            [{"id": f"temperature.{r}", "type": "temperature", "room": r,
              "unit": "C"} for r in hm.ROOMS]
            + [{"id": "temperature.outside", "type": "temperature",
                "room": "outside", "unit": "C"}]
            + [{"id": f"motion.{r}", "type": "motion", "room": r}
               for r in hm.ROOMS]
        )
        return {
            "ok": True,
            "robot_id": bridge.robot_id,
            "model": "OmniLink Smart House",
            "class": "smart_home_hub",
            "dof": 0,
            "tick_period_s": bridge.timestep / 1000.0,
            "actions": list(HUB_VERBS),
            "sensors": sensors,
            "rooms": list(hm.ROOMS),
            "devices": list(hm.ALL_DEVICE_IDS),
            "time": {
                "time_scale": model.time_scale,
                "house_time": bridge.house_time_cache,
                "note": ("house_time advances with the simulator: 1 sim "
                         "second = time_scale/60 house minutes. Command "
                         "results are measured after the write has landed "
                         "on an engine step, never echoed."),
            },
            "notes": [
                "set_device/toggle_device/set_scene answer with MEASURED "
                "post-settle state (PROTOCOL.md 5.4.1); thermostat, lock "
                "and alarm state come from the hub registry, which is "
                "their ground truth.",
                "check_anomalies detects door_open_while_armed and "
                f"energy_spike (> {int(hm.ENERGY_SPIKE_WATTS)} W sustained "
                f"{int(hm.ENERGY_SPIKE_SUSTAIN_HMIN)} house-min while the "
                "occupant is away) only.",
                "set_schedule / shut_water_main / shut_gas_main are "
                "honestly rejected: this hub does not implement them.",
            ],
        }

    def state_body() -> Dict[str, Any]:
        return {
            "ok": True,
            "robot_id": bridge.robot_id,
            "sim_time": round(bridge.sim_now, 3),
            "last_tick_at": round(bridge.sim_now, 3),
            "mode": "hub",
            "fault": None,
            "state_source": "sim",
            "house_time": bridge.house_time_cache,
            "scenario": model.scenario,
            "resident": model.resident,
        }

    # ---- hub verb handlers (each returns a JSON-able body) ----------

    def h_list_rooms(body):
        def run():
            counts = {r: 0 for r in hm.ROOMS}
            for i in hm.ALL_DEVICE_IDS:
                r = hm.DEVICE_ROOM.get(i)
                if r in counts:
                    counts[r] += 1
            return [{"id": r, "name": hm.ROOM_NAMES[r],
                     "device_count": counts[r]} for r in hm.ROOMS]
        return bridge.executor.submit(run)

    def device_entry(dev_id: str) -> Dict[str, Any]:
        state = bridge.measure_device(dev_id)
        entry = {
            "id": dev_id,
            "type": hm.DEVICE_TYPE[dev_id],
            "room_id": hm.DEVICE_ROOM[dev_id],
            "state": state,
            "capabilities": CAPABILITIES_BY_TYPE[hm.DEVICE_TYPE[dev_id]],
            "online": bridge.scene.is_online(dev_id),
        }
        if dev_id == "appliance.coffee_maker":
            entry["coffee_ready"] = bridge.model.coffee_ready
        return entry

    def h_list_devices(body):
        room = body.get("room")

        def run():
            if room is not None and room not in hm.ROOMS:
                return fault("room_unknown",
                             f"unknown room {room!r}; rooms: {hm.ROOMS}")
            out = []
            for i in hm.ALL_DEVICE_IDS:
                if room is not None and hm.DEVICE_ROOM[i] != room:
                    continue
                out.append(device_entry(i))
            return out
        return bridge.executor.submit(run)

    def h_read_sensors(body):
        return bridge.executor.submit(lambda: model.read_sensors(body.get("room")))

    def h_get_device_state(body):
        dev = body.get("id")

        def run():
            if dev not in hm.ALL_DEVICE_IDS:
                return fault("device_offline", f"unknown device id {dev!r}")
            meta = model.meta[dev]
            resp = {
                "id": dev,
                "state": bridge.measure_device(dev),
                "last_change": meta["last_change"],
                "changed_by": meta["changed_by"],
                "online": bridge.scene.is_online(dev),
            }
            if dev == "thermostat.main":
                resp["mode"] = model.devices[dev]["mode"]
                resp["heater"] = bridge.measure_device("hvac.heater")
            if dev == "appliance.coffee_maker":
                resp["coffee_ready"] = model.coffee_ready
            return resp
        return bridge.executor.submit(run)

    def h_set_device(body):
        dev, state = body.get("id"), body.get("state")
        return bridge.transact(
            lambda: model.apply_set_device(dev, state, "agent"),
            dev_ids_for_read=dev if isinstance(dev, str) else (lambda: {}))

    def h_toggle_device(body):
        dev = body.get("id")

        def read():
            return {"new_state": bridge.measure_device(dev)}
        return bridge.transact(
            lambda: model.apply_toggle_device(dev, "agent"),
            dev_ids_for_read=read)

    def h_set_scene(body):
        scene_name = body.get("scene")
        affected_ids: List[str] = []

        def apply():
            err, affected = model.apply_scene(scene_name, "agent")
            if err is not None:
                return err
            affected_ids.extend(affected)
            return None

        def read():
            return {"scene": scene_name,
                    "affected": [{"id": i, "state": bridge.measure_device(i)}
                                 for i in affected_ids]}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_adjust_thermostat(body):
        dev = body.get("id", "thermostat.main")
        target, mode = body.get("target"), body.get("mode")
        clamp_box = {"clamped": False}

        def apply():
            if dev != "thermostat.main":
                return fault("device_offline",
                             f"{dev!r} is not a thermostat; use thermostat.main")
            err, clamped = model.apply_adjust_thermostat(target, mode, "agent")
            clamp_box["clamped"] = clamped
            return err

        def read():
            st = model.devices["thermostat.main"]
            return {"target": st["target"], "mode": st["mode"],
                    "clamped": clamp_box["clamped"],
                    "heater": bridge.measure_device("hvac.heater")}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_set_schedule(body):
        return {"accepted": False, "error": "state_rejected",
                "message": "this hub does not execute device schedules"}

    def h_lock_door(body):
        dev = body.get("id", "lock.front_door")

        def apply():
            if dev != "lock.front_door":
                return fault("device_offline", f"unknown lock {dev!r}")
            model._commit(dev, "locked", "agent")
            return None

        def read():
            return {"state": model.devices["lock.front_door"]}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_unlock_door(body):
        dev = body.get("id", "lock.front_door")
        auth = body.get("authorization")

        def apply():
            if dev != "lock.front_door":
                return fault("device_offline", f"unknown lock {dev!r}")
            if not (isinstance(auth, str) and auth.strip()):
                return fault("authorization_required",
                             "unlock_door requires an 'authorization' token")
            model._commit(dev, "unlocked", "agent")
            return None

        def read():
            return {"state": model.devices["lock.front_door"]}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_arm_security(body):
        def apply():
            model._commit("security.system", "armed", "agent")
            return None

        def read():
            return {"state": model.devices["security.system"],
                    "zone": body.get("zone", "all")}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_disarm_security(body):
        auth = body.get("authorization")

        def apply():
            if not (isinstance(auth, str) and auth.strip()):
                return fault("authorization_required",
                             "disarm_security requires an 'authorization' token")
            model._commit("security.system", "disarmed", "agent")
            return None

        def read():
            return {"state": model.devices["security.system"]}
        return bridge.transact(apply, dev_ids_for_read=read)

    def h_get_energy_report(body):
        return bridge.executor.submit(
            lambda: model.energy_report(body.get("range", "all")))

    def h_check_anomalies(body):
        return bridge.executor.submit(lambda: {
            "active": list(model.anomalies_active),
            "history": list(model.anomalies_history),
            "house_time": model.house_time(),
        })

    def h_get_weather(body):
        def run():
            hours = (model.clock_hmin / 60.0) % 24.0
            return {"temp_c": round(model.outside_temp(), 2),
                    "condition": "clear" if 6.0 <= hours < 20.5 else "clear-night",
                    "house_time": model.house_time(),
                    "location": body.get("location", "sim_house")}
        return bridge.executor.submit(run)

    def h_notify_occupant(body):
        msg = body.get("message")
        if not isinstance(msg, str) or not msg.strip():
            return fault("state_rejected", "'message' must be a non-empty string")
        return bridge.executor.submit(
            lambda: model.notify(msg.strip(), body.get("severity"),
                                 body.get("channel"), body.get("occupant_id")))

    def h_not_plumbed(body):
        return {"accepted": False, "error": "state_rejected",
                "message": "not plumbed in this house"}

    HUB_HANDLERS = {
        "list_rooms": h_list_rooms,
        "list_devices": h_list_devices,
        "read_sensors": h_read_sensors,
        "get_device_state": h_get_device_state,
        "set_device": h_set_device,
        "toggle_device": h_toggle_device,
        "set_scene": h_set_scene,
        "adjust_thermostat": h_adjust_thermostat,
        "set_schedule": h_set_schedule,
        "lock_door": h_lock_door,
        "unlock_door": h_unlock_door,
        "arm_security": h_arm_security,
        "disarm_security": h_disarm_security,
        "get_energy_report": h_get_energy_report,
        "check_anomalies": h_check_anomalies,
        "get_weather": h_get_weather,
        "notify_occupant": h_notify_occupant,
        "shut_water_main": h_not_plumbed,
        "shut_gas_main": h_not_plumbed,
    }

    # ---- scenario namespace (benchmark-only surface; never agent tools) ----

    def s_start(body):
        name = body.get("name")

        def apply():
            return model.scenario_start(name, body.get("seed"),
                                        body.get("time_scale"),
                                        body.get("start_time"),
                                        body.get("hold_clock", True))

        def read():
            return {"ok": True, "name": model.scenario,
                    "house_time": model.house_time(),
                    "time_scale": model.time_scale,
                    "resident": model.resident,
                    "clock": "held" if model.clock_held else "free_running"}
        return bridge.transact(apply, dev_ids_for_read=read)

    def s_status(body):
        return bridge.executor.submit(model.scenario_status)

    def s_resident(body):
        room = body.get("room")

        def apply():
            if room not in hm.ROOMS and room != "away":
                return fault("room_unknown",
                             f"unknown room {room!r}; rooms: "
                             f"{hm.ROOMS + ['away']}")
            model._set_resident(room, "scenario")
            return None

        def read():
            pos = bridge.scene.read_resident_position()
            measured = hm.room_from_position(*pos) if pos else None
            return {"ok": True, "resident": measured,
                    "commanded": room,
                    "measured_from": "scene" if pos else "unmeasured"}
        return bridge.transact(apply, dev_ids_for_read=read)

    def s_event(body):
        def apply():
            return model.scenario_event(body)

        def read():
            dev = body.get("id")
            out = {"ok": True}
            if isinstance(dev, str) and dev in hm.ALL_DEVICE_IDS:
                out["realized_state"] = bridge.measure_device(dev)
            if body.get("type") == "resident":
                pos = bridge.scene.read_resident_position()
                out["resident"] = hm.room_from_position(*pos) if pos else None
            return out
        return bridge.transact(apply, dev_ids_for_read=read)

    def s_metrics(body):
        return bridge.executor.submit(model.metrics)

    def s_advance(body):
        minutes = body.get("house_minutes")
        if isinstance(minutes, bool) or not isinstance(minutes, (int, float)) \
                or not math.isfinite(float(minutes)) or minutes <= 0:
            return fault("state_rejected",
                         "'house_minutes' must be a positive number")
        minutes = float(minutes)
        if minutes > MAX_ADVANCE_HOUSE_MIN:
            return fault("state_rejected",
                         f"advance is capped at {MAX_ADVANCE_HOUSE_MIN:.0f} "
                         "house-minutes per call; chunk longer advances")
        # Registering the waiter and opening the integration window happen
        # in ONE main-thread job (open_advance), so a held clock cannot
        # leak or jump time around the window edges.
        ev = threading.Event()
        start = bridge.executor.submit(lambda: bridge.open_advance(minutes, ev))
        if start.get("error"):
            return start
        # Blocking by design: the caller's own HTTP timeout governs. A
        # worst-case realtime engine covers 480 house-min in 480 s wall.
        if not ev.wait(timeout=900.0):
            bridge.cancel_advance(ev)
            return fault("degraded",
                         "the engine did not reach the advance target in "
                         "900 s wall time")
        return bridge.executor.submit(lambda: {
            "ok": True,
            "house_time": model.house_time(),
            "advanced_house_min": round(model.clock_hmin - start["hmin"], 3),
            "sim_seconds_stepped": round(bridge.sim_now - start["sim"], 3),
            "clock": "held" if model.clock_held else "free_running",
        })

    def s_reset(body):
        def apply():
            model.reset()
            return None

        def read():
            return {"ok": True, "house_time": model.house_time(),
                    "resident": model.resident}
        return bridge.transact(apply, dev_ids_for_read=read)

    SCENARIO_HANDLERS = {
        "/scenario/start": s_start,
        "/scenario/status": s_status,
        "/scenario/resident": s_resident,
        "/scenario/event": s_event,
        "/scenario/metrics": s_metrics,
        "/scenario/advance": s_advance,
        "/scenario/reset": s_reset,
    }

    # ---- PROTOCOL.md conformance verbs -------------------------------

    def p_stop_robot(body):
        def apply():
            model.apply_set_device("appliance.oven", "off", "agent")
            model.apply_set_device("appliance.coffee_maker", "off", "agent")
            model.apply_adjust_thermostat(None, "off", "agent")
            return None

        def read():
            return {
                "ok": True,
                "halted_at": round(bridge.sim_now, 3),
                "safety_stop": {
                    "appliance.oven": bridge.measure_device("appliance.oven"),
                    "appliance.coffee_maker":
                        bridge.measure_device("appliance.coffee_maker"),
                    "hvac.heater": bridge.measure_device("hvac.heater"),
                },
            }
        return bridge.transact(apply, dev_ids_for_read=read)

    def p_action(body):
        action = body.get("action")
        if action not in HUB_HANDLERS:
            raise RequestError(400, "invalid_action",
                               f"unknown action {action!r}; see "
                               "/capabilities actions", {"action": action})
        rest = {k: v for k, v in body.items() if k not in ("action", "robot_id")}
        return HUB_HANDLERS[action](rest)

    class Handler(BaseHTTPRequestHandler):
        server_version = "OmniSimSmartHouse/1.0"
        protocol_version = "HTTP/1.1"
        timeout = 30

        def log_message(self, fmt, *args):
            return

        def _json(self, code, obj):
            try:
                data = json.dumps(obj, default=str,
                                  allow_nan=False).encode("utf-8")
            except ValueError:
                data = json.dumps(_finite(obj), default=str,
                                  allow_nan=False).encode("utf-8")
            if code >= 400:
                self.close_connection = True
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _guard(self):
            check_protocol_version(self.headers)
            self._response_origin = checked_origin(self.headers, trusted_origins)
            check_authorization(self.headers, bridge_token)

        def do_GET(self):
            try:
                self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                if path == "/healthz":
                    return self._json(200, {"ok": True,
                                            "service": "smart_house_bridge",
                                            "sim_time": round(bridge.sim_now, 3)})
                if path == "/protocol":
                    return self._json(200, {
                        "ok": True,
                        "omnisim_wire": WIRE_VERSION,
                        "service": "robot_bridge",
                        "service_versions": {"robot_bridge": "1.0",
                                             "smart_home_hub": "1.0"},
                        "instance": {
                            "name": "smart_house_bridge",
                            "robot_id": bridge.robot_id,
                            "world": bridge.world_path,
                            "pid": os.getpid(),
                        },
                        "extensions": ["x-omnilink-smart-home-hub"],
                    })
                if path == "/capabilities":
                    return self._json(200, capabilities_body())
                if path == "/state":
                    return self._json(200, state_body())
                if path == "/read_mission_brief":
                    return self._json(200, {"ok": True, "brief": MISSION_BRIEF})
                return self._json(404, {"ok": False,
                                        "error": "unknown_endpoint",
                                        "path": path})
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:  # noqa: BLE001
                self._error(e)

        def do_POST(self):
            try:
                self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                body = read_json(self, allow_empty=True)
                # Idempotency key. PROTOCOL.md spells it 'id', but on this
                # hub 'id' is the DEVICE id in most payloads (Haven's
                # adapters.md), so 'request_id' is the canonical key here
                # and 'id' doubles as one only on paths whose payloads
                # never carry a device id.
                request_id = validate_request_id(body.pop("request_id", None))
                if request_id is None and path not in DEVICE_ID_PATHS:
                    request_id = validate_request_id(body.pop("id", None))
                if path not in READONLY_PATHS:
                    request_ids.claim(path, request_id)

                if path == "/list_robots" or path == "/capabilities":
                    return self._json(200, capabilities_body())
                if path == "/get_robot_state" or path == "/state":
                    return self._json(200, state_body())
                if path == "/stop_robot":
                    return self._json(200, p_stop_robot(body))
                if path == "/reset_to_home":
                    return self._json(200, s_reset(body))
                if path == "/action":
                    return self._json(200, p_action(body))
                if path in SCENARIO_HANDLERS:
                    return self._json(200, SCENARIO_HANDLERS[path](body))
                verb = path.lstrip("/")
                if verb in HUB_HANDLERS:
                    return self._json(200, HUB_HANDLERS[verb](body))
                return self._json(404, {"ok": False,
                                        "error": "unknown_endpoint",
                                        "path": path})
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:  # noqa: BLE001
                self._error(e)

        def _error(self, e):
            import traceback
            print(f"{TAG} HTTP {self.command} {self.path} failed: {e!r}\n"
                  f"{traceback.format_exc()}", flush=True)
            try:
                self._json(500, error_envelope(
                    "internal_error", "The hub could not complete the request."))
            except Exception:  # noqa: BLE001
                pass

    return Handler


# ── Entry point ───────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="smart_house")
    p.add_argument("--port", type=int, default=8766)
    args, _ = p.parse_known_args()
    return args


def main() -> int:
    args = _parse_args()
    sup = Supervisor()
    bridge = SmartHouseBridge(sup, args.robot, args.port)
    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 make_handler(bridge))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"{TAG} hub ready as '{args.robot}' on http://127.0.0.1:{args.port} "
          f"(time_scale={bridge.model.time_scale:g}, "
          f"house_time={bridge.model.house_time()})", flush=True)
    if bridge.scene.missing:
        print(f"{TAG} degraded devices: "
              f"{sorted(bridge.scene._offline)}", flush=True)

    while sup.step(bridge.timestep) != -1:
        bridge.tick()
    return 0


if __name__ == "__main__":
    sys.exit(main())
