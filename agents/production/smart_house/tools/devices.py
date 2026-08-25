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

"""Smart-house device and sensor tools — thin HTTP proxies to the hub.

The 19 Haven-shaped tools (same names, same tiers, same refusal semantics as
OmniLink's Haven agent) pointed at the OmniSim smart-house hub: the
``smart_house_bridge`` controller inside ``omnilink_smart_house.omniworld``,
default ``http://127.0.0.1:8766`` (override with ``SMART_HOUSE_HUB_URL``).

Every tool forwards a JSON payload to the hub and returns the hub's response
as-is (with a defensive wrapper for transport errors). The hub owns the
physics: states in responses are MEASURED from the simulated house, never
echoes of the argument. Tools stay thin — anomaly reasoning, routines, and
preference logic belong in the system prompt, not in a tool wrapper.

Tier semantics (see ``_base.py``): CONFIRM_REQUIRED impls refuse CLIENT-SIDE
when no ``authorization`` token is supplied — the request never reaches the
hub, exactly like Haven.
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from ._base import (
    ALWAYS,
    CONFIRM_REQUIRED,
    GUARDED,
    SAFE,
    ToolSpec,
    hub_call,
)


# ── Sensors / telemetry (safe) ──────────────────────────────────────

def _impl_list_rooms(**_: Any) -> Dict[str, Any]:
    return hub_call("list_rooms")


def _impl_list_devices(room: Optional[str] = None, **_: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if room:
        payload["room"] = room
    return hub_call("list_devices", payload)


def _impl_read_sensors(room: Optional[str] = None, **_: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if room:
        payload["room"] = room
    return hub_call("read_sensors", payload)


def _impl_get_device_state(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return hub_call("get_device_state", {"id": id})


def _impl_check_anomalies(**_: Any) -> Dict[str, Any]:
    return hub_call("check_anomalies")


def _impl_get_energy_report(range: str = "24h", **_: Any) -> Dict[str, Any]:
    return hub_call("get_energy_report", {"range": range})


def _impl_get_weather(location: Optional[str] = None, **_: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if location:
        payload["location"] = location
    return hub_call("get_weather", payload)


# ── Device control (guarded) ────────────────────────────────────────

def _impl_set_device(id: str = "", state: Any = None, **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if state is None:
        return {"error": "state is required"}
    return hub_call("set_device", {"id": id, "state": state})


def _impl_toggle_device(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return hub_call("toggle_device", {"id": id})


def _impl_set_scene(scene: str = "", **_: Any) -> Dict[str, Any]:
    if not scene:
        return {"error": "scene is required"}
    return hub_call("set_scene", {"scene": scene})


def _impl_adjust_thermostat(
    id: str = "",
    target: Optional[float] = None,
    mode: Optional[str] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if target is None and not mode:
        return {"error": "at least one of target or mode is required"}
    payload: Dict[str, Any] = {"id": id}
    if target is not None:
        payload["target"] = target
    if mode:
        payload["mode"] = mode
    return hub_call("adjust_thermostat", payload)


def _impl_set_schedule(id: str = "", schedule: Any = None, **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if schedule is None:
        return {"error": "schedule is required"}
    return hub_call("set_schedule", {"id": id, "schedule": schedule})


# ── Security tools — mixed tiers ────────────────────────────────────

def _impl_lock_door(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return hub_call("lock_door", {"id": id})


def _impl_unlock_door(id: str = "", authorization: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not authorization:
        return {
            "error": "authorization required",
            "hint": "unlock_door needs an occupant-provided authorization token. Ask the occupant to confirm.",
        }
    return hub_call("unlock_door", {"id": id, "authorization": authorization})


def _impl_arm_security(zone: Optional[str] = None, **_: Any) -> Dict[str, Any]:
    payload: Dict[str, Any] = {}
    if zone:
        payload["zone"] = zone
    return hub_call("arm_security", payload)


def _impl_disarm_security(
    zone: Optional[str] = None,
    authorization: str = "",
    **_: Any,
) -> Dict[str, Any]:
    if not authorization:
        return {
            "error": "authorization required",
            "hint": "disarm_security needs an occupant-provided authorization token.",
        }
    payload: Dict[str, Any] = {"authorization": authorization}
    if zone:
        payload["zone"] = zone
    return hub_call("disarm_security", payload)


def _impl_shut_water_main(authorization: str = "", **_: Any) -> Dict[str, Any]:
    if not authorization:
        return {
            "error": "authorization required",
            "hint": "shut_water_main is destructive and needs occupant authorization — even during an active leak alert, propose and wait.",
        }
    return hub_call("shut_water_main", {"authorization": authorization})


def _impl_shut_gas_main(authorization: str = "", **_: Any) -> Dict[str, Any]:
    if not authorization:
        return {
            "error": "authorization required",
            "hint": "shut_gas_main is destructive and needs occupant authorization — even during an active gas alert, propose and wait.",
        }
    return hub_call("shut_gas_main", {"authorization": authorization})


# ── Notification (safe) ─────────────────────────────────────────────

def _impl_notify_occupant(
    message: str = "",
    severity: str = "medium",
    channel: Optional[str] = None,
    occupant_id: Optional[str] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not message:
        return {"error": "message is required"}
    payload: Dict[str, Any] = {"message": message, "severity": severity}
    if channel:
        payload["channel"] = channel
    if occupant_id:
        payload["occupant_id"] = occupant_id
    return hub_call("notify_occupant", payload)


# ── SPECS ────────────────────────────────────────────────────────────

SPECS = [
    # ── Sensing / telemetry ─────────────────────────────────────────
    ToolSpec(
        name="list_rooms",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Enumerate every room the hub knows about. Returns a list of "
            "{id, name, device_count}. This house has four rooms: "
            "living_room, kitchen, bedroom, hallway."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_list_rooms,
        tags=["home", "discovery"],
    ),
    ToolSpec(
        name="list_devices",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "List devices, optionally scoped to a room. Returns each "
            "device's id, type, room_id, current state, and capabilities. "
            "This is how you notice something is in a state it should not "
            "be in (e.g. a heat-producing appliance on while nobody is "
            "home) — the hub reports states, YOU infer what is wrong."
        ),
        parameters={
            "type": "object",
            "properties": {
                "room": {"type": "string", "description": "Optional room id to scope the list."},
            },
        },
        impl=_impl_list_devices,
        tags=["home", "discovery"],
    ),
    ToolSpec(
        name="read_sensors",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read current sensor values: per-room temperature (C) and "
            "motion, plus the outside temperature (room_id 'outside'). "
            "Optionally scope to one room; without `room`, returns the "
            "full sweep. ALWAYS call this at the start of a wake turn "
            "before making any comfort/efficiency decisions."
        ),
        parameters={
            "type": "object",
            "properties": {
                "room": {"type": "string", "description": "Optional room id."},
            },
        },
        impl=_impl_read_sensors,
        tags=["home", "telemetry"],
    ),
    ToolSpec(
        name="get_device_state",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Full state snapshot for one device: current state, mode, "
            "last_change, changed_by, online flag. Use before set_device / "
            "toggle_device to confirm the change is actually needed, and "
            "to see WHO changed a device last (agent, occupant, hub)."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_get_device_state,
        tags=["home", "telemetry"],
    ),
    ToolSpec(
        name="check_anomalies",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Inspect the hub's anomaly queue (open door while armed, "
            "energy spikes). Returns {active, history}. Call at the start "
            "of every wake turn so alerts surface before routine work. "
            "NOTE: the hub only flags what its rules can see — a device "
            "merely left in a wasteful or unsafe state is YOURS to catch "
            "via list_devices + read_sensors."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_check_anomalies,
        tags=["home", "anomaly"],
    ),
    ToolSpec(
        name="get_energy_report",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Aggregate energy usage over a time range. Returns "
            "{total_kwh, by_category: {hvac, lighting, appliances, "
            "standby}, outliers}, integrated over simulated house time. "
            "Default range is '24h'. An appliances outlier while the "
            "house is empty is a red flag worth investigating."
        ),
        parameters={
            "type": "object",
            "properties": {
                "range": {"type": "string", "description": "Time range: '24h', '7d', 'month'.", "default": "24h"},
            },
        },
        impl=_impl_get_energy_report,
        tags=["home", "energy"],
    ),
    ToolSpec(
        name="get_weather",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Current outside conditions: {temp_c, condition, house_time}. "
            "Used for heating decisions (e.g. eco setback depth) and "
            "morning briefs."
        ),
        parameters={
            "type": "object",
            "properties": {
                "location": {"type": "string", "description": "Optional location override; defaults to the house's location."},
            },
        },
        impl=_impl_get_weather,
        tags=["home", "weather"],
    ),

    # ── Device control ──────────────────────────────────────────────
    ToolSpec(
        name="set_device",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Set a device to a target state. Lights accept 'on', 'off', or "
            "{on: bool, brightness: 0-100}; switches accept 'on'/'off'; "
            "blinds 'open'/'closed'. The hub validates against device "
            "capabilities and returns {accepted, realized_state} MEASURED "
            "from the house after the change settles — never an echo of "
            "your argument. If realized_state disagrees with what you "
            "asked for, believe realized_state."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "state": {"description": "Target state — shape varies by device type ('on', 'off', {on, brightness}, 'open', 'closed')."},
            },
            "required": ["id", "state"],
        },
        impl=_impl_set_device,
        tags=["home", "device"],
    ),
    ToolSpec(
        name="toggle_device",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Toggle a binary device (light, switch) between on and off. "
            "Returns {accepted, new_state} (measured). Convenience wrapper "
            "— use set_device for anything non-binary, and the dedicated "
            "lock/security verbs for locks and the alarm."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_toggle_device,
        tags=["home", "device"],
    ),
    ToolSpec(
        name="set_scene",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Apply a named scene: 'morning', 'goodnight', 'movie', 'away'. "
            "Returns {accepted, affected: [{id, state}]} with measured "
            "post-settle states. 'away' = thermostat eco, all lights off, "
            "security armed, front door locked — the standard "
            "leaving-home transition."
        ),
        parameters={
            "type": "object",
            "properties": {"scene": {"type": "string", "description": "Scene name: 'morning', 'goodnight', 'movie', 'away'."}},
            "required": ["scene"],
        },
        impl=_impl_set_scene,
        tags=["home", "scene", "routine"],
    ),
    ToolSpec(
        name="adjust_thermostat",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Change the thermostat's target temperature and/or mode. Pass "
            "at least one of `target` (C) or `mode` ('heat', 'eco', "
            "'off'). The hub clamps target to 5-30 C and flags the clamp. "
            "In 'eco' the house holds target minus 4 — the economical "
            "away/night setting. The single zone thermostat is "
            "thermostat.main."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "target": {"type": "number", "description": "Target temperature in C."},
                "mode": {"type": "string", "description": "Mode: heat, eco, off."},
            },
            "required": ["id"],
        },
        impl=_impl_adjust_thermostat,
        tags=["home", "hvac"],
    ),
    ToolSpec(
        name="set_schedule",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Update a device's built-in schedule. NOTE: this hub does not "
            "execute device schedules and will honestly refuse "
            "(state_rejected). Prefer acting directly on your wake turns; "
            "do not claim a schedule was set when the hub refused."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "schedule": {"description": "Device-specific schedule object."},
            },
            "required": ["id", "schedule"],
        },
        impl=_impl_set_schedule,
        tags=["home", "schedule"],
    ),

    # ── Security — mixed tiers ──────────────────────────────────────
    ToolSpec(
        name="lock_door",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Lock a door (lock.front_door). ALWAYS ALLOWED — tightening "
            "posture is safe. Idempotent. Use during goodnight/away "
            "transitions or in response to an unlocked-door observation."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_lock_door,
        tags=["home", "security"],
    ),
    ToolSpec(
        name="unlock_door",
        tier=CONFIRM_REQUIRED,
        surface=ALWAYS,
        description=(
            "Unlock a door. REQUIRES OCCUPANT AUTHORIZATION — pass an "
            "`authorization` token obtained from an explicit occupant "
            "confirmation turn. Never call from a wake turn or routine; "
            "only from a direct occupant request. If authorization is "
            "missing, the tool returns an error rather than unlocking."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "authorization": {"type": "string", "description": "Occupant-provided authorization token."},
            },
            "required": ["id", "authorization"],
        },
        impl=_impl_unlock_door,
        tags=["home", "security"],
    ),
    ToolSpec(
        name="arm_security",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Arm the security system, optionally scoped to a zone "
            "('perimeter' = front door + windows). ALWAYS ALLOWED — "
            "tightening posture is safe. Use during goodnight and "
            "away-mode transitions."
        ),
        parameters={
            "type": "object",
            "properties": {
                "zone": {"type": "string", "description": "Optional zone id, e.g. 'perimeter'."},
            },
        },
        impl=_impl_arm_security,
        tags=["home", "security"],
    ),
    ToolSpec(
        name="disarm_security",
        tier=CONFIRM_REQUIRED,
        surface=ALWAYS,
        description=(
            "Disarm the security system. REQUIRES OCCUPANT AUTHORIZATION. "
            "If authorization is missing, returns an error. Never disarm "
            "from a wake turn."
        ),
        parameters={
            "type": "object",
            "properties": {
                "zone": {"type": "string"},
                "authorization": {"type": "string", "description": "Occupant-provided authorization token."},
            },
            "required": ["authorization"],
        },
        impl=_impl_disarm_security,
        tags=["home", "security"],
    ),
    ToolSpec(
        name="shut_water_main",
        tier=CONFIRM_REQUIRED,
        surface=ALWAYS,
        description=(
            "Close the main water shutoff valve. DESTRUCTIVE if triggered "
            "in error. REQUIRES OCCUPANT AUTHORIZATION even during an "
            "active leak alert: propose the action and wait. (This house "
            "reports it as not plumbed — expect an honest refusal from "
            "the hub even with authorization.)"
        ),
        parameters={
            "type": "object",
            "properties": {
                "authorization": {"type": "string", "description": "Occupant-provided authorization token."},
            },
            "required": ["authorization"],
        },
        impl=_impl_shut_water_main,
        tags=["home", "safety", "destructive"],
    ),
    ToolSpec(
        name="shut_gas_main",
        tier=CONFIRM_REQUIRED,
        surface=ALWAYS,
        description=(
            "Close the main gas shutoff valve. DESTRUCTIVE if triggered "
            "in error. REQUIRES OCCUPANT AUTHORIZATION even during an "
            "active gas alert: propose and wait. (This house reports it "
            "as not plumbed — expect an honest refusal from the hub even "
            "with authorization.)"
        ),
        parameters={
            "type": "object",
            "properties": {
                "authorization": {"type": "string", "description": "Occupant-provided authorization token."},
            },
            "required": ["authorization"],
        },
        impl=_impl_shut_gas_main,
        tags=["home", "safety", "destructive"],
    ),

    # ── Notification ────────────────────────────────────────────────
    ToolSpec(
        name="notify_occupant",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Send a message to the occupant. Severity: critical (act-now "
            "safety), high (needs attention soon), medium (in-app "
            "summary), low (daily digest). Batch non-critical findings "
            "into ONE notification per wake turn — one summary is better "
            "than five pings. Delivered in-sim; recorded in the metrics "
            "log."
        ),
        parameters={
            "type": "object",
            "properties": {
                "message": {"type": "string"},
                "severity": {
                    "type": "string",
                    "enum": ["critical", "high", "medium", "low"],
                    "default": "medium",
                },
                "channel": {"type": "string", "description": "Optional channel override."},
                "occupant_id": {"type": "string", "description": "Optional target occupant id; defaults to primary."},
            },
            "required": ["message"],
        },
        impl=_impl_notify_occupant,
        tags=["home", "notification"],
    ),
]
