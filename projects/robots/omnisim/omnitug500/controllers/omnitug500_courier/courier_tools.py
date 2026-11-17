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

"""OmniLink tool surface for the warehouse courier.

When OMNI_KEY is set, these Tool definitions are registered with the OmniLink
relay so the cloud agent (Gemini / GPT / Grok / Claude) can drive the rover by
calling them. They map 1:1 onto CourierBridge's thread-safe enqueue actions.
The agent issues a tool call, the bridge enqueues the step, and the agent polls
get_courier_state to follow progress — exactly the loop the production agent
under agents/production/omnitug500_warehouse/ runs.
"""

from __future__ import annotations

from typing import Any, List, Optional


def build_courier_tools(bridge, Tool) -> List[Any]:
    if Tool is None:
        return []
    bay_names = bridge.station_names("pickup")
    dock_names = bridge.station_names("dropoff")
    return [
        Tool(
            name="list_stations",
            description=("List the warehouse pickup bays (with their colours and "
                         "which package is staged at each), the dropoff docks, and "
                         "the rover's deck capacity. Call this first to learn the map."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.capabilities(),
        ),
        Tool(
            name="get_courier_state",
            description=("Read the rover's live status: position, mode "
                         "(idle/drive/align/act), what it is carrying on the deck, "
                         "queued steps, the station it is parked at, and the last "
                         "event. Poll this to know when a step has finished."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.get_state(),
        ),
        Tool(
            name="goto_station",
            description=("Route through the aisles and park at a named station "
                         "(a pickup bay, a dock, or 'home'). Asynchronous: it "
                         "returns immediately; poll get_courier_state until mode "
                         "is 'idle' at that station."),
            parameters={
                "type": "object",
                "properties": {"station": {
                    "type": "string",
                    "description": "Station name, e.g. " + ", ".join(
                        bay_names + dock_names + ["home"]),
                }},
                "required": ["station"],
            },
            dispatch=lambda args: bridge.act_goto(args.get("station")),
        ),
        Tool(
            name="pick_package",
            description=("Drive to a pickup bay and load its staged package onto the "
                         "deck. Give either the bay name or the package name; the "
                         "rover resolves the other. Fails if the deck is full."),
            parameters={
                "type": "object",
                "properties": {
                    "station": {"type": "string",
                                "description": "Pickup bay, e.g. " + ", ".join(bay_names)},
                    "package": {"type": "string",
                                "description": "Optional package name to load."},
                },
            },
            dispatch=lambda args: bridge.act_pick(args.get("station"), args.get("package")),
        ),
        Tool(
            name="deliver_package",
            description=("Drive to a dock and set down the deck's package(s). With no "
                         "'package' it delivers everything currently on the deck."),
            parameters={
                "type": "object",
                "properties": {
                    "station": {"type": "string",
                                "description": "Dropoff dock, e.g. " + ", ".join(dock_names)},
                    "package": {"type": "string",
                                "description": "Optional single package to deliver."},
                },
                "required": ["station"],
            },
            dispatch=lambda args: bridge.act_deliver(args.get("station"), args.get("package")),
        ),
        Tool(
            name="run_route",
            description=("Queue a whole multi-stop courier run in one call. Each step "
                         "is {action: 'pick'|'deliver'|'goto', station: <name>, "
                         "package?: <name>}. Use this for 'collect from A and C then "
                         "deliver to dock 2' style requests."),
            parameters={
                "type": "object",
                "properties": {"steps": {
                    "type": "array",
                    "description": "Ordered list of route steps.",
                    "items": {
                        "type": "object",
                        "properties": {
                            "action": {"type": "string", "enum": ["pick", "deliver", "goto"]},
                            "station": {"type": "string"},
                            "package": {"type": "string"},
                        },
                        "required": ["action", "station"],
                    },
                }},
                "required": ["steps"],
            },
            dispatch=lambda args: bridge.act_run_route(args.get("steps") or []),
        ),
        Tool(
            name="stop_rover",
            description="Emergency halt — stop the rover and clear the queued route.",
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_stop(),
        ),
        Tool(
            name="reset_demo",
            description=("Return the rover to the charging dock and restore every "
                         "package to its starting bay."),
            parameters={"type": "object", "properties": {}},
            dispatch=lambda args: bridge.act_reset(),
        ),
    ]


def build_courier_main_task(bridge) -> str:
    caps = bridge.capabilities()
    bays = "; ".join(f"{b['name']} ({_color(bridge, b['name'])} package)"
                     for b in caps["pickup_bays"])
    docks = ", ".join(d["name"] for d in caps["docks"])
    return (
        "You are the dispatcher brain of a OmniTug 500 autonomous warehouse "
        "courier (an AGV with a flat cargo deck, no arm). An operator tells you, "
        "in plain language, which package to pick from which bay and which dock to "
        "deliver it to. You translate that into tool calls.\n\n"
        f"Pickup bays: {bays}.\n"
        f"Dropoff docks: {docks}. Plus 'home' (the charging dock).\n"
        f"The deck holds up to {caps['deck_capacity']} packages at once.\n\n"
        "Rules:\n"
        "- Start an unfamiliar request with list_stations to confirm names/colours.\n"
        "- For a single pick+deliver, you can call pick_package then deliver_package, "
        "or just run_route([{action:'pick',...},{action:'deliver',...}]).\n"
        "- For multi-stop ('collect from A and C, deliver to dock 2'), use ONE "
        "run_route with a pick step per bay and a final deliver step.\n"
        "- Actions are asynchronous. After issuing them, poll get_courier_state until "
        "mode is 'idle' and the queue is empty before declaring success. The "
        "'carrying' and 'last_event' fields tell you what happened.\n"
        "- 'stop' halts and clears the route; 'reset' restores the demo.\n"
        "- Keep replies to one or two short sentences."
    )


def _color(bridge, name: str) -> str:
    st = bridge.stations.get(name, {})
    return st.get("color_name", "")
