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

"""Courier tools — the agent's action surface over the omnitug500_courier bridge.

Each is a thin wrapper over one bridge endpoint. The bridge owns routing,
collision-safe driving, the deck carry, and the mission queue; the agent just
decides WHAT to do and reads state back to know when a step finished.
"""

from __future__ import annotations

from typing import Any, Dict, List

from ._base import GUARDED, SAFE, ToolSpec, bridge_get, bridge_post


def _list_stations(**_) -> Dict[str, Any]:
    return bridge_get("capabilities")


def _get_state(**_) -> Dict[str, Any]:
    return bridge_get("get_robot_state")


def _goto(station: str = "", **_) -> Dict[str, Any]:
    return bridge_post("goto_station", {"station": station})


def _pick(station: str = "", package: str = "", **_) -> Dict[str, Any]:
    body: Dict[str, Any] = {}
    if station:
        body["station"] = station
    if package:
        body["package"] = package
    return bridge_post("pick_package", body)


def _deliver(station: str = "", package: str = "", **_) -> Dict[str, Any]:
    body: Dict[str, Any] = {"station": station}
    if package:
        body["package"] = package
    return bridge_post("deliver_package", body)


def _run_route(steps: List[Dict[str, Any]] = None, **_) -> Dict[str, Any]:
    return bridge_post("run_route", {"steps": steps or []})


def _stop(**_) -> Dict[str, Any]:
    return bridge_post("stop")


def _reset(**_) -> Dict[str, Any]:
    return bridge_post("reset")


_STATION = {"type": "string", "description": "Station name (bay-a..bay-f, dock-1..dock-3, home)."}

SPECS = [
    ToolSpec(
        name="list_stations", tier=SAFE,
        description=("List the warehouse pickup bays (name, colour, staged package), "
                     "the dropoff docks, and the rover's deck capacity. Call first to "
                     "learn the map before planning a route."),
        parameters={"type": "object", "properties": {}},
        impl=_list_stations, tags=["read"]),
    ToolSpec(
        name="get_courier_state", tier=SAFE,
        description=("Live rover status: position, mode (idle/drive/align/act), what is "
                     "on the deck (carrying), queued steps, the station it is parked at, "
                     "and the last event. Poll until mode is 'idle' and queue is 0 to "
                     "confirm a step finished."),
        parameters={"type": "object", "properties": {}},
        impl=_get_state, tags=["read"]),
    ToolSpec(
        name="goto_station", tier=GUARDED,
        description=("Route through the aisles and park at a named station. "
                     "Asynchronous — poll get_courier_state until idle there."),
        parameters={"type": "object", "properties": {"station": _STATION},
                    "required": ["station"]},
        impl=_goto, tags=["motion"]),
    ToolSpec(
        name="pick_package", tier=GUARDED,
        description=("Drive to a pickup bay and load its staged package onto the deck. "
                     "Give the bay name or the package name. Fails if the deck is full."),
        parameters={"type": "object", "properties": {
            "station": {"type": "string", "description": "Pickup bay (bay-a..bay-f)."},
            "package": {"type": "string", "description": "Optional package name."}}},
        impl=_pick, tags=["motion"]),
    ToolSpec(
        name="deliver_package", tier=GUARDED,
        description=("Drive to a dock and set down the deck's package(s). With no "
                     "'package' it delivers everything on the deck."),
        parameters={"type": "object", "properties": {
            "station": {"type": "string", "description": "Dropoff dock (dock-1..dock-3)."},
            "package": {"type": "string", "description": "Optional single package."}},
            "required": ["station"]},
        impl=_deliver, tags=["motion"]),
    ToolSpec(
        name="run_route", tier=GUARDED,
        description=("Queue a whole multi-stop run in one call. steps is an ordered list "
                     "of {action:'pick'|'deliver'|'goto', station:<name>, package?:<name>}. "
                     "Use for 'collect from A and C then deliver to dock 2'."),
        parameters={"type": "object", "properties": {"steps": {
            "type": "array", "description": "Ordered route steps.",
            "items": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["pick", "deliver", "goto"]},
                "station": {"type": "string"}, "package": {"type": "string"}},
                "required": ["action", "station"]}}},
            "required": ["steps"]},
        impl=_run_route, tags=["motion"]),
    ToolSpec(
        name="stop_rover", tier=GUARDED,
        description="Emergency halt — stop the rover and clear the queued route.",
        parameters={"type": "object", "properties": {}},
        impl=_stop, tags=["motion"]),
    ToolSpec(
        name="reset_demo", tier=GUARDED,
        description="Return the rover home and restore every package to its starting bay.",
        parameters={"type": "object", "properties": {}},
        impl=_reset, tags=["motion"]),
]
